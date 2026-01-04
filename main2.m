clc; 
clear all; 

% Initialize state and EKF variables
dt = 0.1; % Time step
state = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0]; % Initial state [px, py, pz, vx, vy, vz, q1, q2, q3, q4]
P = eye(10); % Initial covariance matrix
Q = eye(10) * 0.1; % Process noise covariance
R = eye(6) * 0.05; % Measurement noise covariance (GPS/IMU)
target_state = [5; 5; 5; 0; 0; 0; 1; 0; 0; 0]; % Target state

% RL parameters (using a simple Q-learning approach)
Q_table = zeros(100, 4); % Discretized state-action space
alpha = 0.1; % Learning rate
gamma = 0.9; % Discount factor
epsilon = 0.1; % Exploration rate

% Data storage for plotting
P_R = [];          % Covariance of position and velocities
xhatR = [];        % Filtered state (position, velocity, quaternion, etc.)
OMEGA = [];        % Bias-free angular velocities
OMEGA_raw = [];    % Raw angular velocities
FX = [];           % Accelerometer readings
gps_data = [];     % Array to store GPS measurements over time
imu_data = [];     % Array to store IMU measurements over time

% Main loop
for episode = 1:1000
    for t = 1:100
        % 1. EKF Predict Step
        F = eye(10); % Simplified state transition model (Jacobian)
        [x_pred, P_pred] = ekf_predict(state, P, F, Q);
        
        % 2. Get GPS and IMU measurements (with noise)
        gps_meas = state(1:3) + randn(3, 1) * 0.1; % GPS position measurement
        imu_meas = state(4:6) + randn(3, 1) * 0.01; % IMU velocity measurement
        
        % Store state and covariance data for plotting
        P_R = [P_R; diag(P)'];                    % Diagonal covariance for px, py, pz, etc.
        xhatR = [xhatR; state'];                  % State estimate [px, py, pz, vx, vy, vz, q1, q2, q3, q4]
        OMEGA = [OMEGA; state(4:6)'];             % Angular velocities (bias-free, e.g., vx, vy, vz)
        OMEGA_raw = [OMEGA_raw; imu_meas'];       % Raw IMU data [p, q, r]
        
        gps_data = [gps_data; gps_meas'];    % Store GPS measurements
        imu_data = [imu_data; imu_meas'];    % Store IMU velocity measurements
        
        % Break if target state is reached
        if norm(state(1:3) - target_state(1:3)) < 0.1
            break;
        end
        
        % 3. EKF Update Step
        z = [gps_meas; imu_meas];
        H = [eye(6) zeros(6, 4)]; % Measurement matrix (GPS + IMU)
        K = ekf_gain(P_pred, H, R);
        [state, P] = ekf_update(x_pred, P_pred, z, H, R, K);
        
        % 4. RL Control Action
        state_index = discretize_state(state);
        state_index = max(1, min(100, state_index)); % Clamp state_index to [1, 100]

        if rand < epsilon
            % Explore: Random action
            control_action = randi([1, 4], 1, 1);
        else
            % Exploit: Choose best action from Q-table
            [~, control_action] = max(Q_table(state_index, :));
        end
        
        % 5. Apply control action (e.g., thrust)
        control_input = action_to_input(control_action);
        next_state = drone_dynamics(state, control_input, dt);
        
        % 6. Reward calculation
        reward = calculate_reward(next_state, target_state);
        
        % 7. Update Q-table
        next_state_index = discretize_state(next_state);
        next_state_index = max(1, min(100, next_state_index)); % Clamp next_state_index to [1, 100]

        best_next_action = max(Q_table(next_state_index, :));
        Q_table(state_index, control_action) = Q_table(state_index, control_action) + ...
            alpha * (reward + gamma * best_next_action - Q_table(state_index, control_action));
        
        % 8. Move to next state
        state = next_state;
        
        % Break if target state is reached
        if norm(state(1:3) - target_state(1:3)) < 0.1
            break;
        end
    end
    
    % Decrease exploration rate over time
    epsilon = max(0.01, epsilon * 0.99);
end
