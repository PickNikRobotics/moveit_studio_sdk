# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for the caller metadata attached by ObjectiveManager."""

import uuid
from unittest.mock import MagicMock

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_sdk_msgs.msg import ObjectiveCaller

import moveit_studio_py.objective_manager as objective_manager_module
from moveit_studio_py.objective_manager import ObjectiveManager


def make_manager(monkeypatch, **manager_kwargs):
    """Construct an ObjectiveManager with mocked node/executor seams.

    Returns the manager plus the mocked execute and cancel service clients so
    tests can inspect the requests sent through the public API.
    """
    execute_client = MagicMock()
    execute_client.wait_for_service.return_value = True
    execute_response = MagicMock()
    execute_response.error_code.val = MoveItErrorCodes.SUCCESS
    execute_client.call.return_value = execute_response

    cancel_client = MagicMock()
    cancel_client.wait_for_service.return_value = True
    cancel_response = MagicMock()
    cancel_response.status.success = True
    cancel_client.call.return_value = cancel_response

    node = MagicMock()
    node.create_client.side_effect = [execute_client, cancel_client]
    monkeypatch.setattr(
        objective_manager_module, "create_node", lambda *args, **kwargs: node
    )
    monkeypatch.setattr(
        objective_manager_module.executors,
        "MultiThreadedExecutor",
        lambda: MagicMock(),
    )

    return ObjectiveManager(**manager_kwargs), execute_client, cancel_client


def test_caller_metadata_is_stable_across_start_and_stop(monkeypatch) -> None:
    """The declared caller identity rides every start and stop request, with a
    valid UUIDv4 instance ID."""
    manager, execute_client, cancel_client = make_manager(
        monkeypatch,
        caller_type=ObjectiveCaller.SCRIPT,
        caller_name="objective_manager_test",
    )
    assert manager.start_objective("Test Objective") == (True, "")
    manager.stop_objective()

    start_caller = execute_client.call.call_args.args[0].caller
    stop_caller = cancel_client.call.call_args.args[0].caller
    assert start_caller.type == ObjectiveCaller.SCRIPT
    assert start_caller.name == "objective_manager_test"
    assert uuid.UUID(start_caller.instance_id).version == 4
    assert stop_caller == start_caller


def test_caller_type_defaults_to_unknown(monkeypatch) -> None:
    """Without explicit caller arguments the request declares UNKNOWN/empty,
    and managers in the same process share one instance ID."""
    manager, execute_client, _ = make_manager(monkeypatch)
    second_manager, second_execute_client, _ = make_manager(monkeypatch)
    assert manager.start_objective("Test Objective") == (True, "")
    assert second_manager.start_objective("Test Objective") == (True, "")

    caller = execute_client.call.call_args.args[0].caller
    second_caller = second_execute_client.call.call_args.args[0].caller
    assert caller.type == ObjectiveCaller.UNKNOWN
    assert caller.name == ""
    assert uuid.UUID(caller.instance_id).version == 4
    assert second_caller.instance_id == caller.instance_id


def test_caller_instance_id_is_resolved_at_request_time(monkeypatch) -> None:
    """A manager inherited across fork() must stamp the child's UUID: the
    instance ID is read per request, not captured at construction."""
    manager, execute_client, _ = make_manager(
        monkeypatch, caller_type=ObjectiveCaller.SCRIPT, caller_name="fork_test"
    )
    child_uuid = str(uuid.uuid4())
    monkeypatch.setattr(
        objective_manager_module, "_process_instance_id", lambda: child_uuid
    )
    assert manager.start_objective("Test Objective") == (True, "")

    caller = execute_client.call.call_args.args[0].caller
    assert caller.instance_id == child_uuid
    assert caller.type == ObjectiveCaller.SCRIPT
    assert caller.name == "fork_test"
