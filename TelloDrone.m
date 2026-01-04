clc;
clear all; 

% Initialize and connect to Tello drone
drone = ryze(); % Connects to the Tello drone
takeoff(drone); % Tello drone takes off

% Move commands
moveforward(drone, 'Distance', 1); % Move forward by 1 meter
pause(1); % Pause for 1 second
moveright(drone, 'Distance', 0.5); % Move right by 0.5 meter
pause(1);
moveup(drone, 'Distance', 0.5); % Move up by 0.5 meter
pause(1);
moveback(drone, 'Distance', 1); % Move back by 1 meter
pause(1);

% Rotate commands
turn(drone, deg2rad(90)); % Turn 90 degrees to the right
pause(1);
turn(drone, deg2rad(-90)); % Turn 90 degrees to the left
pause(1);

% Land the drone
land(drone); % Tello drone lands
