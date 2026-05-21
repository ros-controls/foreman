import flatbuffers
import foreman.fbs.ComponentState
import foreman.fbs.ErrorState
import foreman.fbs.ForemanSnapshot
from foreman.types import ForemanSnapshot

def serialize_foreman_snapshot(snapshot: ForemanSnapshot) -> bytearray:
    """
    Converts the strongly-typed ForemanSnapshot into a FlatBuffer bytearray.
    """
    builder = flatbuffers.Builder(1024)

    # deepest structures first! ComponentState and ErrorState list of components
    # saving offsets for strings, or other complex types (tables, lists...). Basic types can be added directly.
    err_comp_offsets = [builder.CreateString(c) for c in snapshot.error.components]
    
    foreman.fbs.ErrorState.StartComponentsVector(builder, len(err_comp_offsets))
    # push in REVERSE!
    for offset in reversed(err_comp_offsets): 
        builder.PrependUOffsetTRelative(offset)
    err_comp_vec_offset = builder.EndVector()

    err_msg_offset = builder.CreateString(snapshot.error.message)
    err_category_offset = builder.CreateString(snapshot.error.category)

    comp_offsets = []
    for comp in snapshot.components:
        name_offset = builder.CreateString(comp.name)
        state_offset = builder.CreateString(comp.lifecycle_state.name)
        type_offset = builder.CreateString(comp.component_type.value)
        
        foreman.fbs.ComponentState.Start(builder)
        foreman.fbs.ComponentState.AddName(builder, name_offset)
        foreman.fbs.ComponentState.AddState(builder, state_offset)
        foreman.fbs.ComponentState.AddComponentType(builder, type_offset)
        comp_offsets.append(foreman.fbs.ComponentState.End(builder)) 
    
    foreman.fbs.ForemanSnapshot.StartComponentsVector(builder, len(comp_offsets))
    for offset in reversed(comp_offsets):
        builder.PrependUOffsetTRelative(offset)
    main_comps_vec_offset = builder.EndVector()

    goal_offset = builder.CreateString(snapshot.goal)

    # Then we build the parent tables: ErrorState and ForemanSnapshot
    foreman.fbs.ErrorState.Start(builder)
    foreman.fbs.ErrorState.AddIsError(builder, snapshot.error.is_error)
    foreman.fbs.ErrorState.AddCategory(builder, err_category_offset)
        
    foreman.fbs.ErrorState.AddMessage(builder, err_msg_offset)
    foreman.fbs.ErrorState.AddComponents(builder, err_comp_vec_offset)
    error_state_offset = foreman.fbs.ErrorState.End(builder)

    foreman.fbs.ForemanSnapshot.Start(builder)
    foreman.fbs.ForemanSnapshot.AddGoal(builder, goal_offset)
    foreman.fbs.ForemanSnapshot.AddReady(builder, snapshot.ready)
    foreman.fbs.ForemanSnapshot.AddAtGoal(builder, snapshot.at_goal)
    foreman.fbs.ForemanSnapshot.AddError(builder, error_state_offset)
    foreman.fbs.ForemanSnapshot.AddComponents(builder, main_comps_vec_offset)
    foreman_offset = foreman.fbs.ForemanSnapshot.End(builder)

    builder.Finish(foreman_offset)
    return builder.Output()