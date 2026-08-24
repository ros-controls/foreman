# Current Foreman engine status.

# Profile Foreman is driving toward, or "None" if none is requested yet.
string target_profile
# Profile the live observed state matches, or "None" if none matches.
string current_profile
bool ready
bool at_profile
ForemanErrorState error
# Every profile name declared in scenario.yaml.
string[] all_profiles
# Subset of all_profiles currently achievable given observed component state.
string[] available_profiles
