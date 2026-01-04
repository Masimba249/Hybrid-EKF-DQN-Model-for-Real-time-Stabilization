function [nextObs, reward, isDone, LoggedSignal] = droneStepFunction(action, LoggedSignal)
    % Minimal step function logic for testing
    nextObs = zeros(10, 1);  % Placeholder for next observation
    reward = 0;               % Placeholder reward
    isDone = false;           % Placeholder termination flag
    LoggedSignal.State = nextObs;  % Update LoggedSignal structure
end
