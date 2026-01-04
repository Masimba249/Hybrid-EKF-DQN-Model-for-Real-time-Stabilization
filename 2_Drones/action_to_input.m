function control_input = action_to_input(action)
    control_input = zeros(3, 1);
    switch action
        case 1
            control_input = [1; 0; 0];
        case 2
            control_input = [-1; 0; 0];
        case 3
            control_input = [0; 1; 0];
        case 4
            control_input = [0; 0; 1];
        otherwise
            error('Invalid action. Action must be between 1 and 4.');
    end
end
