# Message containing a classification hypothesis.

# Identifier of the hypothesized class.
# This is a freeform ID. If using a class hierarchy, use / as category separator.
string class_id

# The probability or confidence value of this hypothesis.
# Should be within [0, 1]. The higher the score, the more likely the hypothesis.
float64 score