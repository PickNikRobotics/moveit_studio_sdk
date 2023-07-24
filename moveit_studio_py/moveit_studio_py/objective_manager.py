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

import threading
import rclpy

from collections.abc import Callable
from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_sdk_msgs.srv import CancelObjective, ExecuteObjective


class ObjectiveManager:
    """
    Provides a high-level API for starting/stopping Objectives.
    Internally, this class takes care of communicating with the Objective Server.
    """

    __EXECUTE_OBJECTIVE_SERVICE = "/execute_objective"
    __CANCEL_OBJECTIVE_SERVICE = "/cancel_objective"

    def __init__(self):
        """
        Constructor.
        """
        self._node = rclpy.create_node("moveit_studio_objective_manager")

        self._execute_objective_client = self._node.create_client(
            ExecuteObjective, self.__EXECUTE_OBJECTIVE_SERVICE
        )
        while not self._execute_objective_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(
                self.__EXECUTE_OBJECTIVE_SERVICE
                + " service not available, waiting again..."
            )

        self._cancel_objective_client = self._node.create_client(
            CancelObjective, self.__CANCEL_OBJECTIVE_SERVICE
        )
        while not self._cancel_objective_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(
                self.__CANCEL_OBJECTIVE_SERVICE
                + " service not available, waiting again..."
            )

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._executor_thread.start()

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
        blocking: bool = True,
        async_callback: Callable[[rclpy.task.Future], None] = None,
    ) -> tuple[bool, str]:
        """
        Run an Objective.

        Args:
            objective_name: the (string) name of the Objective to run.
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
        # TODO(adlarkin) support parameter_overrides:
        # https://github.com/PickNikRobotics/moveit_studio/blob/main/src/moveit_studio_msgs/moveit_studio_agent_msgs/srv/ExecuteObjective.srv#L7
        if blocking:
            result = self._execute_objective_client.call(request)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return (True, "")
            elif result.error_message:
                return (False, result.error_message)
            else:
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
        result = self._cancel_objective_client.call(request)
        if not result.status.success:
            self._node.get_logger().warn(result.status.error_message)
