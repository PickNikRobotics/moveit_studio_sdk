# BT.CPP UIDs for one BehaviorTree subtree, in DFS document order.

# The `<BehaviorTree ID="...">` this group belongs to.
string tree_id

# UIDs in DFS document order. BT.CPP `TreeNode::UID()` is `uint16_t`.
uint16[] uids
