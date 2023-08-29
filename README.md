# MoveIt Studio SDK

This repository provides an SDK that can be used with MoveIt Studio.
The SDK can be used in the MoveIt Studio Agent container, or separately on a ROS 2 enabled system.
To test using the SDK and MoveIt Studio separately, take a look at the Docker setup below.

The full documentation for MoveIt Studio and the MoveIt Studio SDK is hosted at https://docs.picknik.ai/en/stable/index.html.

## Docker setup for testing the MoveIt Studio SDK in a separate container

The Docker workflow outlined here provides a way to test the MoveIt Studio SDK in isolation from MoveIt Studio.

First, build the image if you haven't already.
This image will install the MoveIt Studio SDK for you:

```
docker compose build
```

Start MoveIt Studio if it isn't running already, making note of the `ROS_DOMAIN_ID` that's being used.
The MoveIt Studio SDK Docker image being used here expects MoveIt Studio to use the default Cyclone DDS configuration.

Next, set the `ROS_DOMAIN_ID` environment variable in the terminal where you plan to start the MoveIt Studio SDK container.
The `ROS_DOMAIN_ID` being set here should match the `ROS_DOMAIN_ID` you are using for MoveIt Studio.

Once the Docker image is built and the `ROS_DOMAIN_ID` is set, start a container:

```
docker compose run --rm moveit_studio_sdk_standalone
```

You should now be at a terminal inside of the SDK container.
To ensure that the SDK container can communicate with MoveIt Studio, run:

```
ros2 service list
```

You should see a list of MoveIt Studio services.

Let's now test the SDK to see if we can execute and cancel MoveIt Studio Objectives.
There are three example scripts provided:
* `start_blocking.py`: starts an Objective and waits until the Objective finishes execution.
* `start_stop_async.py`: starts an Objective, waits for a user-defined number of seconds, and then cancels the running Objective.
* `move_to_waypoint.py`: runs the "Move to Waypoint" Objective, using a parameter override to set the waypoint to move to.

To run the `Open Gripper` Objective, let's use `start_blocking.py` since this Objective shouldn't take long to complete:

```
ros2 run moveit_studio_py start_blocking.py "Open Gripper"
```

To run the `3 Waypoints Pick and Place` Objective, let's use `start_stop_async.py` since this Objective runs indefinitely.
We will let this Objective run for 10 seconds:

```
ros2 run moveit_studio_py start_stop_async.py "3 Waypoints Pick and Place" 10
```

To run the `move_to_waypoint.py` example, a valid waypoint must be provided.
A list of pre-defined waypoints can be found in the ui's "Endpoint" tab.
The following command would move the robot arm to the `Look at Left Table` waypoint:

```
ros2 run moveit_studio_py move_to_waypoint.py "Look at Left Table"
```

If you look at the `Objectives` tab in the MoveIt Studio UI, you should see these Objectives starting/stopping successfully.
