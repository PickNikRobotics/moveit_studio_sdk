# MoveIt Pro SDK

This repository provides the ROS 2 message and action definitions used by MoveIt Pro, enabling programmatic interaction with the MoveIt Pro runtime.

## Branches in the Public Mirror

`moveit_pro/src/sdk` is the source of truth. Release-tagged snapshots are
copied to the public `moveit_studio_sdk` repository as `release/<tag>`; use the
release branch matching your MoveIt Pro version. The public repository's `main`
branch is a legacy development line and is not a version-parity mirror.

## Message Definitions

The SDK includes message definitions for:

- `DoObjectiveSequence` action for executing MoveIt Pro Objectives
- Error codes and status messages
- Objective execution feedback and results

## Using the SDK

For detailed instructions on how to use these message definitions to interact with MoveIt Pro programmatically, please refer to [Tutorial 2: Runtime SDK & Extensions](https://docs.picknik.ai/en/stable/tutorials/developer_platform_usage.html).

The tutorial covers:

- **Native ROS Client Library API**: Using `rclpy`/`rclcpp` to interact directly with MoveIt Pro via ROS 2 actions
- **Objective execution and parameter overrides**: Using the typed MoveIt Pro ROS 2 interfaces from native clients

For the full MoveIt Pro documentation, visit <https://docs.picknik.ai/en/stable/index.html>.
