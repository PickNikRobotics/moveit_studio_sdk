# Copyright 2023 Picknik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Picknik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import threading
import uuid
from rclpy import task, executors, create_node
from typing import Optional

from collections.abc import Callable
from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_sdk_msgs.msg import BehaviorParameter, ObjectiveCaller
from moveit_studio_sdk_msgs.srv import CancelObjective, ExecuteObjective

_process_instance: Optional[tuple[int, str]] = None
_process_instance_lock = threading.Lock()


def _reset_process_instance_after_fork() -> None:
    """Reinitialize fork-inherited state in the child.

    A child forked while another thread held the lock would inherit it locked and
    deadlock on the first _process_instance_id() call.
    """
    global _process_instance, _process_instance_lock
    _process_instance_lock = threading.Lock()
    _process_instance = None


os.register_at_fork(after_in_child=_reset_process_instance_after_fork)


def _process_instance_id() -> str:
    """Return a UUID unique to the current process.

    Cached per PID rather than at import time, so processes forked after import
    (e.g. via the ``fork`` multiprocessing start method) each get their own ID.
    """
    global _process_instance
    pid = os.getpid()
    with _process_instance_lock:
        if _process_instance is None or _process_instance[0] != pid:
            _process_instance = (pid, str(uuid.uuid4()))
        return _process_instance[1]


class ObjectiveManager:
    """
    Provides a high-level API for starting/stopping Objectives.
    Internally, this class takes care of communicating with the Objective Server.
    """

    __EXECUTE_OBJECTIVE_SERVICE = "/execute_objective"
    __CANCEL_OBJECTIVE_SERVICE = "/cancel_objective"

    def __init__(
        self,
        caller_type: int = ObjectiveCaller.UNKNOWN,
        caller_name: str = "",
    ):
        """
        Constructor.

        Args:
            caller_type: Caller-declared client type. The SDK cannot infer whether
                         it is used by a script or an external application, so the
                         integrator should set this explicitly. Defaults to UNKNOWN.
            caller_name: Human-readable client name, such as ``amp``.
        """
        self._caller_type = caller_type
        self._caller_name = caller_name
        self._node = create_node(
            "moveit_studio_objective_manager", parameter_overrides=[]
        )

        self._execute_objective_client = self._node.create_client(
            ExecuteObjective, self.__EXECUTE_OBJECTIVE_SERVICE
        )
        if not self._execute_objective_client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(
                f"{self.__EXECUTE_OBJECTIVE_SERVICE} service not available."
            )

        self._cancel_objective_client = self._node.create_client(
            CancelObjective, self.__CANCEL_OBJECTIVE_SERVICE
        )
        if not self._cancel_objective_client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(
                f"{self.__CANCEL_OBJECTIVE_SERVICE} service not available."
            )

        self._executor = executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._executor_thread.start()

    def _make_caller(self) -> ObjectiveCaller:
        """Build the caller per request so a manager inherited across a fork()
        stamps the child's process UUID, not the parent's."""
        return ObjectiveCaller(
            type=self._caller_type,
            name=self._caller_name,
            instance_id=_process_instance_id(),
        )

    def __del__(self):
        """
        Destructor.
        """
        if hasattr(self, "_executor"):
            self._executor.shutdown()
        if hasattr(self, "_executor_thread"):
            self._executor_thread.join()

    def start_objective(
        self,
        objective_name: str,
        parameter_overrides: Optional[list[BehaviorParameter]] = None,
        blocking: bool = True,
        async_callback: Optional[Callable[[task.Future], None]] = None,
    ) -> tuple[bool, str]:
        """
        Run an Objective.

        Args:
            objective_name: the (string) name of the Objective to run.
            parameter_overrides: Parameters to pass to the Objective, if any. The Objective will use its default values for parameters that aren't defined in this list.
            blocking: Whether this method call should block until Objective execution is complete or not. For long-running Objectives,
                      users can set this to False and make use of async_callback to get details about the execution result of the Objective.
            async_callback: A method that is triggered when Objective execution is done.
                            This is only used if blocking is False.

        Returns:
            A tuple of (bool, string) which defines if the Objective executed successfully or not.
            For blocking=true:
              * If Objective execution succeeded, the bool is True and the string is empty.
              * If Objective execution failed, the bool is False and the string explains why execution failed.
            For blocking=false, the bool is True and the string is empty. This indicates that the non-blocking call has been initiated.
            Use the async_callback parameter to get details about the execution result of the non-blocking Objective.
        """
        request = ExecuteObjective.Request()
        request.objective_name = objective_name
        request.parameter_overrides = parameter_overrides if parameter_overrides else []
        request.caller = self._make_caller()
        if blocking:
            result = self._execute_objective_client.call(request)
            if result is None:
                return (False, "The execute-objective service returned no response.")
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return (True, "")
            elif result.error_code.val == MoveItErrorCodes.PREEMPTED:
                print("Objective STOPPED by user")
                return (True, "")
            else:
                print("Objective FAILED")
                return (False, f"MoveItErrorCode Value: {result.error_code.val}")
        else:
            if not async_callback:
                return (
                    False,
                    "No done callback was defined, so the Objective was not triggered asynchronously.",
                )
            future = self._execute_objective_client.call_async(request)
            future.add_done_callback(async_callback)
            return (True, "")

    def stop_objective(self) -> None:
        """
        Stop the Objective that's currently running.
        If no Objectives are currently running, this method does nothing.
        """
        request = CancelObjective.Request()
        request.caller = self._make_caller()
        result = self._cancel_objective_client.call(request)
        if result is None:
            self._node.get_logger().error(
                "The cancel-objective service returned no response."
            )
            return
        if not result.status.success:
            self._node.get_logger().warn(result.status.error_message)
