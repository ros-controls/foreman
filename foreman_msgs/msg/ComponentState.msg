# A component and the state it was observed in.

string name
# ComponentType value: hardware, controller or lifecycle_node.
# Empty when the component is no longer observed, e.g. it vanished from
# /activity, so the name is still reported without a known state.
string component_type
string lifecycle_state
