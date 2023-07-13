# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import argparse
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
        self._node = rclpy.create_node("moveit_studio_objective_manager")
        self._execute_objective_client = self._node.create_client(
            ExecuteObjective, self.__EXECUTE_OBJECTIVE_SERVICE
        )
        self._cancel_objective_client = self._node.create_client(
            CancelObjective, self.__CANCEL_OBJECTIVE_SERVICE
        )

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._executor_thread.start()

    def __del__(self):
        self._executor.shutdown()
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
        self._execute_objective_client.wait_for_service()
        if blocking:
            result = self._execute_objective_client.call(request)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return (True, "")
            elif hasattr(result.error_code, "error_message"):
                return (False, result.error_code.error_message)
            else:
                return (False, f"MoveItErrorCode Value: {result.error_code.val}")
        else:
            future = self._execute_objective_client.call_async(request)
            if async_callback:
                future.add_done_callback(async_callback)
            return (True, "")

    def stop_objective(self) -> None:
        """
        Stop the Objective that's currently running.
        If no Objectives are currently running, this method does nothing.
        """
        request = CancelObjective.Request()
        self._cancel_objective_client.wait_for_service()
        self._cancel_objective_client.call(request)
