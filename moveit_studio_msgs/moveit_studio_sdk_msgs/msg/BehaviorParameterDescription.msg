# Message type that defines the name and type of a Behavior parameter.

# The name of the Behavior parameter.
string name

# The possible types that the Behavior parameter can have, indicated by the `TYPE_*` fields.
# The parameter can only have one value set at any given moment.
# The current value type is specified via the `type` field.
uint8 TYPE_STRING = 0
uint8 TYPE_POSE = 1
uint8 TYPE_DOUBLE = 2
uint8 TYPE_INT = 3
uint8 TYPE_BOOL = 4
uint8 TYPE_JOINT_STATE = 5
uint8 type
