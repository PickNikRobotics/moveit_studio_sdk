import rclpy
from moveit_studio_py.objective_manager import ObjectiveManager


def main():
    rclpy.init()

    print("creating ObjectiveManager")
    objective_manager = ObjectiveManager()

    print("starting blocking objective")
    result = objective_manager.start_objective("Open Gripper", blocking=True)
    print(result)

    print(
        "stopping the objective (this shouldn't do anything b/c blocking in original call)"
    )
    objective_manager.stop_objective()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
