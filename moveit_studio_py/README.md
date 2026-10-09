# Python API for MoveIt Pro

This package provides an [`ObjectiveManager`](moveit_studio_py/objective_manager.py) Python class which can be used to control MoveIt Pro Objectives.

The `examples` directory has several scripts which show how to run Objectives synchronously, asynchronously, and with parameter overrides.

Applications should declare their caller identity when constructing an
`ObjectiveManager`. The manager generates one UUID for the Python process and
uses the same caller metadata on every start and stop request:

```python
from moveit_studio_sdk_msgs.msg import ObjectiveCaller
from moveit_studio_py.objective_manager import ObjectiveManager

objective_manager = ObjectiveManager(
    caller_type=ObjectiveCaller.EXTERNAL_APPLICATION,
    caller_name="amp",
)
```

The caller type defaults to `ObjectiveCaller.UNKNOWN` because the SDK cannot
infer whether it is embedded in a script or an external application.

When using the `fork` `multiprocessing` start method, a process forked after
import gets its own UUID rather than inheriting the parent's, so requests from
forked workers are attributable to the worker that sent them. The caller type
and name are unchanged by the fork.
