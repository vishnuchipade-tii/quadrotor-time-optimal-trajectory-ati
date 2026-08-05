from core.simulation import (SimMetadata, SimulationResult)

# ---------------------------------------
# SIMULATION LOOP
# ---------------------------------------

def run_simulation(uav, planner, target, sim_metadata: SimMetadata):

    time = sim_metadata.start_time
    dt = sim_metadata.dt
    steps = sim_metadata.num_steps

    planner.plan(uav.state.as_vector(), target, sim_metadata, current_time=time)

    uav.time_history = [time]

    for i in range(steps):

        u_opt = planner.control(time, uav.state.as_vector())

        uav.set_control(u_opt)
        uav.step(dt)

        time += dt
        uav.time_history.append(time)

        # termination condition
        if target.is_reached(uav.state):
            uav.set_control(u_opt)
            print(f"{uav.name} reached target at t = {time:.3f}s")
            break

        if i == steps - 1:
            uav.set_control(u_opt)

        if time>=planner.result.final_time:
            uav.set_control(u_opt)
            break

    return uav.state_history, uav.control_history, uav.time_history