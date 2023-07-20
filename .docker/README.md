# Docker setup for testing the MoveIt Studio SDK in a separate container

The MoveIt Studio SDK can be used on a system that does not have MoveIt Studio running on it.
The Docker setup in this directory provides a way to test this isolation.

First, build the image if you haven't already:

```
docker build -t studio_sdk_standalone:latest .
```

Then, create a MoveIt Studio SDK workspace.
The following steps would create this workspace at `~/sdk_ws`:

```
cd ~
mkdir -p sdk_ws/src
cd sdk_ws/src
git clone https://github.com/PickNikRobotics/moveit_studio_sdk.git
```

You can now start a container that will load the SDK workspace into it as a volume.
This will allow you to build and test the MoveIt SDK inside of the container.
Be sure to set the `ROS_DOMAIN_ID` and `SDK_WS` environment variables.
The `SDK_WS` should be the path to the root of SDK workspace, and `ROS_DOMAIN_ID` should match the `ROS_DOMAIN_ID` you are using for MoveIt Studio:

```
ROS_DOMAIN_ID=<your_domain_id> SDK_WS=~/sdk_ws docker compose run --rm --name studio_sdk sdk_standalone
```

You should now be at the root of the SDK workspace inside of the container.
Make sure that all of the SDK's dependencies are installed:

```
apt update && apt -y upgrade && rosdep install --from-paths src -y --ignore-src
```

Once all of the dependencies are installed, build the SDK workspace:

```
colcon build
```

Start MoveIt Studio if it isn't running already.
Be sure to use the same `ROS_DOMAIN_ID` that was used for the SDK container.

Now, go back to the terminal inside of the SDK container.
To ensure that the SDK container can communicate with the Studio container, run:

```
ros2 service list
```

You should see a list of MoveIt Studio services.

Let's now test the SDK to see if we can control Studio Objectives from this other container.
Source the SDK workspace that was built:

```
source /opt/sdk_ws/install/setup.bash
```

We can now try running Objectives with the SDK.
There are two example scripts provided, `start_blocking.py` and `start_stop_async.py`.

To run the `Open Gripper` Objective:

```
ros2 run moveit_studio_py start_blocking.py "Open Gripper"
```

To run the `3 Waypoints Pick and Place` Objective for 10 seconds:

```
ros2 run moveit_studio_py start_stop_async.py "3 Waypoints Pick and Place" 10
```

If you look at the `Objectives` tab in the MoveIt Studio UI, you should see these Objectives starting/stopping successfully.
