# Reports whether the robot is in a fault state, and whether that fault can be reset without manual intervention.
int8 status

int8 NORMAL=0
int8 RECOVERABLE_FAULT=1
int8 NONRECOVERABLE_FAULT=2
