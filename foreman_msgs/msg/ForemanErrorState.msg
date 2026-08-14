# Domain error reported by the Foreman engine.
# Mirrors foreman.types.ErrorSnapshot.

bool is_error
# ForemanErrorCategory value: TransportError, ExecutionError,
# UnexpectedStateError, PlannerError, or None.
string category
string message
ComponentState[] components
