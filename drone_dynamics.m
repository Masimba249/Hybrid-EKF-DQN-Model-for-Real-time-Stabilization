function next_state = drone_dynamics(state, control_input, dt)
    position = state(1:3);
    velocity = state(4:6);
    position = position + velocity * dt;
    if length(control_input) == 3
        velocity = velocity + control_input * dt;
    else
        error('control_input must be a 3-element vector');
    end
    q_next = state(7:10); % Quaternion, unchanged for simplicity
    next_state = [position; velocity; q_next];
end
