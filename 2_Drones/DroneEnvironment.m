function env = DroneEnvironment(n_states, n_actions)
    % Define observation space
    obsInfo = rlNumericSpec([n_states 1], 'LowerLimit', -inf, 'UpperLimit', inf);
    obsInfo.Name = 'Drone States';

    % Define action space
    actInfo = rlFiniteSetSpec([-1, 0, 1]);
    actInfo.Name = 'Drone Actions';

    % Create environment using function names instead of anonymous functions
    env = rlFunctionEnv(obsInfo, actInfo, "droneStepFunction", "droneResetFunction");
end
