clc;
clear all;

% Load data from CSV
data = readtable('2_Drone_Flight_Time_Data.csv');
num_steps = height(data);

% Define the time step
dt = 0.1; % Time step in seconds

% Separate data for each drone
drone_1_data = data(data.drone_id == 1, :);
drone_2_data = data(data.drone_id == 2, :);

% Stabilization target: maintain a fixed position and zero velocity
target_state = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0]; % Target state for stabilization

% DQN parameters
numActions = 4; % Number of possible actions
stateDim = 10; % Dimension of state vector
actionDim = numActions;
learningRate = 0.001;
discountFactor = 0.99;
epsilon = 0.1; % Exploration rate
epsilonDecay = 0.99;
minEpsilon = 0.01;

% Initialize neural network for DQN
layers = [
    featureInputLayer(stateDim, 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(24, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(24, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(actionDim, 'Name', 'fc3')];
dqnNet = dlnetwork(layerGraph(layers));

% Experience replay buffer
bufferCapacity = 10000;
buffer = struct('states', zeros(bufferCapacity, stateDim), ...
                'actions', zeros(bufferCapacity, 1), ...
                'rewards', zeros(bufferCapacity, 1), ...
                'nextStates', zeros(bufferCapacity, stateDim), ...
                'dones', zeros(bufferCapacity, 1));
bufferIndex = 1;

% Training parameters
batchSize = 64;
numEpisodes = 1000;
maxSteps = min(num_steps, 100); % Limit to available data

% Initialize storage for error metrics
position_rmse = zeros(numEpisodes, 1);
velocity_rmse = zeros(numEpisodes, 1);
orientation_rmse = zeros(numEpisodes, 1);

% Loop over episodes for training the model
for episode = 1:numEpisodes
    position_errors = [];
    velocity_errors = [];
    orientation_errors = [];
    
    for t = 1:maxSteps
        % Select current drone data (alternate between drones in each episode)
        if mod(episode, 2) == 1
            current_data = drone_1_data;
        else
            current_data = drone_2_data;
        end

        % Initialize state with data for the current time step
        state = [current_data.px(t); current_data.py(t); current_data.pz(t); ...
                 current_data.vx(t); current_data.vy(t); current_data.vz(t); ...
                 current_data.q1(t); current_data.q2(t); current_data.q3(t); current_data.q4(t)];
        
        % Set initial covariance matrix, process noise, and measurement noise
        P = eye(10); % Initial covariance matrix
        Q = eye(10) * 0.1; % Process noise covariance
        R = eye(6) * 0.05; % Measurement noise covariance (GPS/IMU)

        % 1. EKF Predict Step
        F = eye(10); % Simplified state transition model (Jacobian)
        [x_pred, P_pred] = ekf_predict(state, P, F, Q);

        % 2. Get GPS and IMU measurements from data
        gps_meas = [current_data.GPS_px(t); current_data.GPS_py(t); current_data.GPS_pz(t)];
        imu_meas = [current_data.IMU_vx(t); current_data.IMU_vy(t); current_data.IMU_vz(t)];
        
        % 3. EKF Update Step
        z = [gps_meas; imu_meas];
        H = [eye(6) zeros(6, 4)]; % Measurement matrix (GPS + IMU)
        K = ekf_gain(P_pred, H, R);
        [state, P] = ekf_update(x_pred, P_pred, z, H, R, K);
        
        % 4. RL Control Action (epsilon-greedy policy)
        if rand < epsilon
            control_action = randi([1, numActions]);
        else
            % Reshape state to column vector and use correct dlarray format
            qValues = predict(dqnNet, dlarray(state, 'CB'));
            [~, control_action] = max(extractdata(qValues));
        end
        
        % 5. Apply control action (e.g., thrust)
        control_input = action_to_input(control_action);
        next_state = drone_dynamics(state, control_input, dt);
        
        % 6. Calculate errors for metrics
        position_error = norm(next_state(1:3) - target_state(1:3));
        velocity_error = norm(next_state(4:6) - target_state(4:6));
        orientation_error = 1 - abs(dot(next_state(7:10), target_state(7:10))); % Dot product for quaternion similarity
        
        % Store errors for metric calculations
        position_errors = [position_errors; position_error];
        velocity_errors = [velocity_errors; velocity_error];
        orientation_errors = [orientation_errors; orientation_error];

        % Move to the next state
        state = next_state;
    end
    
    % Calculate RMSE for each metric at the end of the episode
    position_rmse(episode) = sqrt(mean(position_errors.^2));
    velocity_rmse(episode) = sqrt(mean(velocity_errors.^2));
    orientation_rmse(episode) = sqrt(mean(orientation_errors.^2));

    % Decay exploration rate
    epsilon = max(minEpsilon, epsilon * epsilonDecay);
end


