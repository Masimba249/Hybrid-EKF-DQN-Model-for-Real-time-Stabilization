function [initialObs, LoggedSignal] = droneResetFunction()
    % Initialize observation and LoggedSignal at the start
    initialObs = zeros(10, 1);  % Initial observation
    LoggedSignal = struct();    % Initialize as empty struct or with necessary fields
end
