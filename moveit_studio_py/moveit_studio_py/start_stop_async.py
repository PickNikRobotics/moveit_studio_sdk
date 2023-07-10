import rclpy
import time

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_py.objective_manager import ObjectiveManager


def done_cb(future: rclpy.task.Future) -> None:
    result = future.result()
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        print("Async Objective executed successfully")
    elif hasattr(result.error_code, "error_message"):
        print(f"Error occurred: {result.error_code.error_message}")
    else:
        print(f"MoveItErrorCode Value: {result.error_code.val}")


def main():
    rclpy.init()

    print("creating ObjectiveManager")
    objective_manager = ObjectiveManager()

    print("starting async objective")
    result = objective_manager.start_objective(
        "3 Waypoints Pick and Place", blocking=False, async_callback=done_cb
    )
    print(result)

    time.sleep(2)

    print("stopping the async objective")
    objective_manager.stop_objective()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
