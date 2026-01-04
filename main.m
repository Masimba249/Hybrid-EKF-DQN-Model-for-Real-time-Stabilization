clc; 
clear all; 

n_states = 10;
n_actions = 3;
env = DroneEnvironment(n_states, n_actions);
disp('Environment created successfully.');
