#!/usr/bin/env python3

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

# Run the "Move to Waypoint" Objective with the waypoint defined as a parameter override.

import argparse
import rclpy

from moveit_studio_py.objective_manager import ObjectiveManager
from moveit_studio_sdk_msgs.msg import BehaviorParameter
from moveit_studio_sdk_msgs.msg import BehaviorParameterDescription


def main():
    __OBJECTIVE_NAME = "Move to Waypoint"

    parser = argparse.ArgumentParser()
    parser.add_argument("waypoint_name", type=str, help="Name of waypoint to move to.")
    args = parser.parse_args()

    # define the Objective's target waypoint via parameter override
    waypoint_override = BehaviorParameter()
    waypoint_override.description.name = "waypoint_name"
    waypoint_override.description.type = BehaviorParameterDescription.TYPE_STRING
    waypoint_override.string_value = args.waypoint_name
    waypoint_override.behavior_namespaces = ["move_to_waypoint"]

    rclpy.init()

    print(f"Starting {__OBJECTIVE_NAME}.")
    objective_manager = ObjectiveManager()
    parameter_overrides = [waypoint_override]
    success, error_msg = objective_manager.start_objective(
        __OBJECTIVE_NAME, parameter_overrides=parameter_overrides, blocking=True
    )
    if success:
        print("Objective executed successfully!")
    else:
        print(error_msg)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
