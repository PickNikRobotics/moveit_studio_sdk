import argparse
import rclpy
from moveit_studio_py.objective_manager import ObjectiveManager


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("objective_name", type=str, help="Name of the Objective to start.")

    args = parser.parse_args()

    rclpy.init()

    objective_manager = ObjectiveManager()

    print(f"Starting {args.objective_name}.")
    result = objective_manager.start_objective(args.objective_name, blocking=True)
    if result[0]:
        print("Objective executed successfully!")
    else:
        print(result[1])

    rclpy.shutdown()


if __name__ == "__main__":
    main()
