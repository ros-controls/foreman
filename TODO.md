Todos:

- make the core engine + datatypes.
- test core engine in unit tests with datatypes
- write yaml parsing for the configuration
- create foreman_msgs package
- write ros2 interfaces to connect to controller controller_manager
- define the facade for frontend
- write ros2 interfaces for "cli", connect to the facade
- NO LOADING COMPONENTS load components as well - all of this testing so far assumes we're already loaded the hardware and controllers. now we try and fit in component loading.
- answer... do we want to interrupt current goals with the new goals if we spam goals? Currently, a batch of goals will get executed and all blocks until that happens. Make it so that service_Caller only calls a single command at a time. The planner should track what is the current goal, but get_next_transition should output the next component that transitions.
- order -> C deactivate > HW Down > HW Up > C cleanup > C config > C activate.
- write the first docs to explain the architecture
- figure out how do we propagate error messages from controller manager throughout the core system, so frontend consumes it nicely.
    - we'll do this by exposing and API _log_and_abort_goal() which will update the error message in system snapshot and set the goal state to be 
    the current state, with a name "aborted" and an informative error message.



- consider are we catching the failure states. These have to be available in Foreman get_snapshot() so we can easily expose them in the frontend.
    - if any of the controller_manager service calls failed TO DELIVER (network, timeout), we need to be as infromative as possible.
    - if any of the controller_manager service calls failed TO EXECUTE (the logic), we need to be as infromative as possible.
    - if we unexpectedly ended up in a state that is different than what we expected (the expected reality) - in set_system_state(). For example, we assing goal "running",
        we reach it, and franka crashes to unconfigured while no goals were active. We want to log informative error message, and not automatically try
        and reach the goal, we want to go to "faulted" goal. This should be a part of state_monitor.py
        - this should better be called "error" goal.
    - we communicate informative messages via system-snapshot to the frontend.
        - do we need any system to allow for each of the engine.py api failures to give errors some other way?
        - how do we communicate every type of these errors informatively?
        - We need to think of a comprehensive way that failures of certain adapters, engine or planner can propagate between them. Think of several palusible ways to do this, how is it done in hexagonal architecture. Do we define error objects for each adapter? Do we have them in the engine? Do we just split into DeliveryError, LogicError and RealityError and run with that? how is this usually handled?
    - SOLUTION: we'll define domain types for errors. Adapters will translate errors to and from there. in engine.py the domain errors will be translated to system_snapshot as well for easy GET.
    - WE WON"T be throwing EXCEPTIONS. We'll treat errors as data structures and pass them around.

- related, currently, the API is returning Touple[bool, str]. This is wonky for our facade, we want to have a single return object that all adapters use. That way the adapters decide on how to present those (bools, strings, read system_snapshot etc.)

- do we need to flesh out engine_snapshot more? put current state and some other stuff inside? Can that just be the main "public members" object we return, so in one function call we can output the whole engine state.

- datalayer adapter
