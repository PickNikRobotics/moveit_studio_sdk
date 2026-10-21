# Geometry of a ChArUco calibration board, the target of the camera calibration Behaviors and Objectives.
# Lengths are in meters.
#
# YAML format on a Behavior port (top-level sequence containing a single map; BT.CPP reads any port value of the
# form `{key}` as a blackboard reference, so the top level is a sequence):
#   [{squares_x: 7, squares_y: 5, square_length: 0.04, marker_length: 0.03, dictionary: DICT_4X4_50}]
# `dictionary` is one of the constant names below; its number is accepted too. Every key is required: an empty
# port is an error, since no board is right by default. A board pose needs interior corners in both directions,
# so each side has at least 3 squares, and the dictionary must hold the board's squares_x * squares_y / 2 markers.
# The C++ port parser, the Desktop App board editor, and the REST board renderer each carry the dictionary table
# and the ranges below; keep them in step.

# Chessboard squares along the board x axis, from 3 to 50.
uint32 squares_x
# Chessboard squares along the board y axis, from 3 to 50.
uint32 squares_y
# Side length of one chessboard square, in meters, from 0.001 to 10.
float64 square_length
# Side length of the ArUco marker inside a square, in meters, from 0.001 to 10. Smaller than square_length.
float64 marker_length

# ArUco dictionary of the markers, one of the constants below. The values are OpenCV's predefined dictionary ids.
uint8 dictionary
uint8 DICT_4X4_50=0
uint8 DICT_4X4_100=1
uint8 DICT_4X4_250=2
uint8 DICT_4X4_1000=3
uint8 DICT_5X5_50=4
uint8 DICT_5X5_100=5
uint8 DICT_5X5_250=6
uint8 DICT_5X5_1000=7
uint8 DICT_6X6_50=8
uint8 DICT_6X6_100=9
uint8 DICT_6X6_250=10
uint8 DICT_6X6_1000=11
uint8 DICT_7X7_50=12
uint8 DICT_7X7_100=13
uint8 DICT_7X7_250=14
uint8 DICT_7X7_1000=15
uint8 DICT_ARUCO_ORIGINAL=16
uint8 DICT_APRILTAG_16H5=17
uint8 DICT_APRILTAG_25H9=18
uint8 DICT_APRILTAG_36H10=19
uint8 DICT_APRILTAG_36H11=20
