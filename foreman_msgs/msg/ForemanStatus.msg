# Current Foreman engine status.

string goal
bool ready
bool at_goal
ForemanErrorState error
# Every goal name declared in scenario.yaml.
string[] all_goals
# Subset of all_goals currently achievable given observed component state.
string[] available_goals
