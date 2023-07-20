import argparse
import rclpy
import time

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_py.objective_manager import ObjectiveManager


def done_cb(future: rclpy.task.Future) -> None:
    result = future.result()
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        print("Objective executed successfully!")
    elif hasattr(result.error_code, "error_message"):
        print(f"Error occurred: {result.error_code.error_message}")
    else:
        print(f"MoveItErrorCode Value: {result.error_code.val}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("objective_name", type=str, help="Name of the Objective to start.")
    parser.add_argument("wait_time", type=int, default=0,
        help="Time to wait (in seconds) before cancelling the Objective.")

    args = parser.parse_args()
    if args.wait_time < 0:
        raise ValueError("wait_time must be a value >= 0.")

    rclpy.init()

    objective_manager = ObjectiveManager()

    print(f"Starting {args.objective_name}.")
    result = objective_manager.start_objective(
        args.objective_name, blocking=False, async_callback=done_cb
    )

    time.sleep(args.wait_time)

    print(f"Stopping {args.objective_name}.")
    objective_manager.stop_objective()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
