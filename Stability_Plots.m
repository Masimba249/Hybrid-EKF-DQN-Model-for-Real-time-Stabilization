% Define time vector based on the number of steps
t = 1:maxSteps;

% Placeholder for accumulated state and covariance values
position_drift = zeros(maxSteps, 3);
velocity_drift = zeros(maxSteps, 3);
orientation_stability = zeros(maxSteps, 4);

% For each step, store relevant metrics
for i = 1:maxSteps
    position_drift(i, :) = [state(1), state(2), state(3)];    % Position drift over time
    velocity_drift(i, :) = [state(4), state(5), state(6)];    % Velocity drift over time
    orientation_stability(i, :) = [state(7), state(8), state(9), state(10)]; % Quaternion stability
end

% Plot Position Drift
figure(1)
plot(t, position_drift(:, 1), 'LineWidth', 2);
hold on
plot(t, position_drift(:, 2), 'LineWidth', 2);
plot(t, position_drift(:, 3), 'LineWidth', 2);
title('Position Drift Over Time')
xlabel('Time Steps')
ylabel('Position (m)')
legend('px', 'py', 'pz')
grid on
hold off

% Plot Velocity Drift
figure(2)
plot(t, velocity_drift(:, 1), 'LineWidth', 2);
hold on
plot(t, velocity_drift(:, 2), 'LineWidth', 2);
plot(t, velocity_drift(:, 3), 'LineWidth', 2);
title('Velocity Drift Over Time')
xlabel('Time Steps')
ylabel('Velocity (m/s)')
legend('vx', 'vy', 'vz')
grid on
hold off

% Plot Orientation Stability (Quaternions)
figure(3)
plot(t, orientation_stability(:, 1), 'LineWidth', 2);
hold on
plot(t, orientation_stability(:, 2), 'LineWidth', 2);
plot(t, orientation_stability(:, 3), 'LineWidth', 2);
plot(t, orientation_stability(:, 4), 'LineWidth', 2);
title('Orientation Stability (Quaternion)')
xlabel('Time Steps')
ylabel('Quaternion')
legend('q1', 'q2', 'q3', 'q4')
grid on
hold off
