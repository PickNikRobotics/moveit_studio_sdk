# Caller-declared identity for Objective start and cancellation requests.
#
# IP address and transport are intentionally excluded because callers cannot
# observe those values reliably. Network-edge components own that metadata.

uint8 UNKNOWN=0
uint8 WEB_UI=1
uint8 EXTERNAL_APPLICATION=2
uint8 SCRIPT=3

uint8 type
string name
string instance_id
