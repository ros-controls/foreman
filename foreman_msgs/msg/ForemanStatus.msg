# Current Foreman engine status.

string profile
bool ready
bool at_profile
ForemanErrorState error
# Every profile name declared in scenario.yaml.
string[] all_profiles
# Subset of all_profiles currently achievable given observed component state.
string[] available_profiles
# Machine operating mode, per ISO 10218-1 5.7: Automatic, ManualReducedSpeed, ManualHighSpeed.
string operating_mode
# Machine stop state, per Machinery Regulation (EU) 2023/1230 1.2.4:
# Running, NormalStop, OperationalStop, EmergencyStop.
string stop_state
