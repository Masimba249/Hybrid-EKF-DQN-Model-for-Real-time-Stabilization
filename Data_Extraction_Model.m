% Initialize and connect to Tello drone
drone = ryze(); % Connect to the Tello drone
takeoff(drone); % Tello drone takes off

% Initialize data logging variables
position_log = [];
orientation_log = [];
battery_log = [];

% Define a function to log data
function logData()
    % Retrieve telemetry data
    position = drone.Position; % Get [x, y, z] position (in meters)
    orientation = drone.Orientation; % Get orientation (Euler angles)
    battery = drone.BatteryLevel; % Get battery level (%)

    % Append to logs
    position_log = [position_log; position];
    orientation_log = [orientation_log; orientation];
    battery_log = [battery_log; battery];
end

% Move commands and log data after each movement
moveforward(drone, 'Distance', 1); % Move forward by 1 meter
logData();
pause(1);

moveright(drone, 'Distance', 0.5); % Move right by 0.5 meter
logData();
pause(1);

moveup(drone, 'Distance', 0.5); % Move up by 0.5 meter
logData();
pause(1);

moveback(drone, 'Distance', 1); % Move back by 1 meter
logData();
pause(1);

% Rotate commands and log data after each rotation
turn(drone, deg2rad(90)); % Turn 90 degrees to the right
logData();
pause(1);

turn(drone, deg2rad(-90)); % Turn 90 degrees to the left
logData();
pause(1);

% Land the drone and log final data
land(drone); % Tello drone lands
logData();

% Display logged data
disp('Position Log:');
disp(position_log);

disp('Orientation Log:');
disp(orientation_log);

disp('Battery Log:');
disp(battery_log);

% Save data to a file for later analysis
save('drone_flight_data.mat', 'position_log', 'orientation_log', 'battery_log');
