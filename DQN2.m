clc;
clear all;

% Load data from CSV
data = readtable('Drone_Flight_Data.csv');
num_steps = height(data);

% Initialize state and EKF variables
dt = 0.1; % Time step
state = [data.px(1); data.py(1); data.pz(1); data.vx(1); data.vy(1); data.vz(1); data.q1(1); data.q2(1); data.q3(1); data.q4(1)];
P = eye(10); % Initial covariance matrix
Q = eye(10) * 0.1; % Process noise covariance
R = eye(6) * 0.05; % Measurement noise covariance (GPS/IMU)
target_state = [5; 5; 5; 0; 0; 0; 1; 0; 0; 0]; % Target state

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

for episode = 1:numEpisodes
    for t = 1:maxSteps
        % 1. EKF Predict Step
        F = eye(10); % Simplified state transition model (Jacobian)
        [x_pred, P_pred] = ekf_predict(state, P, F, Q);

        % 2. Get GPS and IMU measurements from data
        gps_meas = [data.GPS_px(t); data.GPS_py(t); data.GPS_pz(t)];
        imu_meas = [data.IMU_vx(t); data.IMU_vy(t); data.IMU_vz(t)];
        
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
        
        % 6. Reward calculation
        reward = calculate_reward(next_state, target_state);
        done = norm(next_state(1:3) - target_state(1:3)) < 0.1;
        
        % 7. Store experience in replay buffer
        buffer.states(bufferIndex, :) = state';
        buffer.actions(bufferIndex) = control_action;
        buffer.rewards(bufferIndex) = reward;
        buffer.nextStates(bufferIndex, :) = next_state';
        buffer.dones(bufferIndex) = done;
        bufferIndex = mod(bufferIndex, bufferCapacity) + 1;
        
        % 8. Sample batch from replay buffer and train DQN
        if bufferIndex > batchSize
            batchIdx = randperm(bufferIndex - 1, batchSize);
            batch = struct(...
                'states', buffer.states(batchIdx, :), ...
                'actions', buffer.actions(batchIdx), ...
                'rewards', buffer.rewards(batchIdx), ...
                'nextStates', buffer.nextStates(batchIdx, :), ...
                'dones', buffer.dones(batchIdx));
            
            % Compute Q-targets for training
            nextQValues = predict(dqnNet, dlarray(batch.nextStates', 'CB'));
            maxNextQ = max(extractdata(nextQValues), [], 1);
            qTargets = batch.rewards' + (1 - batch.dones') .* discountFactor .* maxNextQ;
            
            % Update Q-values for chosen actions
            qValues = predict(dqnNet, dlarray(batch.states', 'CB'));
            for i = 1:batchSize
                qValues(batch.actions(i), i) = qTargets(i);
            end
            
            % Calculate gradients using dlfeval
            [loss, gradients] = dlfeval(@lossFunction, dqnNet, dlarray(batch.states', 'CB'), qValues);

            % Manually update each learnable parameter
            for i = 1:size(dqnNet.Learnables, 1)
                dqnNet.Learnables.Value{i} = dqnNet.Learnables.Value{i} - learningRate * gradients.Value{i};
            end
        end

        % Move to the next state
        state = next_state;
        
        % Break if target state is reached
        if done
            break;
        end
    end
    
    % Decay exploration rate
    epsilon = max(minEpsilon, epsilon * epsilonDecay);
end

% Loss function for DQN update
function [loss, gradients] = lossFunction(dqnNet, states, qValues)
    predictions = predict(dqnNet, states);
    loss = mean((predictions - qValues).^2, 'all');
    loss = dlarray(loss);
    gradients = dlgradient(loss, dqnNet.Learnables);
end
