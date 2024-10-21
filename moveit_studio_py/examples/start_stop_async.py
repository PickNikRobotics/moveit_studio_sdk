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

# Starts an Objective asynchronously, and then stops the running Objective after waiting for a user-defined
# number of seconds.

import argparse
import rclpy
import time

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_py.objective_manager import ObjectiveManager


def done_cb(future: rclpy.task.Future) -> None:
    """
    Callback that is triggered when an Objective that was started asynchronously is done executing.

    Args:
        future: the Objective's future, which contains info about the completion status of the Objective.
    """
    result = future.result()
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        print("Objective executed successfully!")
    else:
        print(f"MoveItErrorCode Value: {result.error_code.val}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "objective_name", type=str, help="Name of the Objective to start."
    )
    parser.add_argument(
        "wait_time",
        type=float,
        default=0.0,
        help="Time to wait (in seconds) before cancelling the Objective.",
    )

    args = parser.parse_args()
    if args.wait_time < 0:
        raise ValueError("wait_time must be a value >= 0.")

    rclpy.init()

    objective_manager = ObjectiveManager()

    print(f"Starting {args.objective_name}.")
    objective_manager.start_objective(
        args.objective_name, blocking=False, async_callback=done_cb
    )

    time.sleep(args.wait_time)

    print(f"Stopping {args.objective_name}.")
    objective_manager.stop_objective()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
