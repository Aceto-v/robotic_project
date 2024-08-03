%%% Prendibot Simulation %%%
clear all
close all
clc

%% Load Functions and Data folders
addpath("functions")
addpath("data")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Define Simulation Parameters
% Time Settings
tf = 1e1;    % [s]
fs = 1e3;    % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-2;

% Init Condition
q0 = q_iniziale';
qdot0 = zeros(length(q0), 1);

%% Generate Trajectory
[q_des, qdot_des, q2dot_des] = jtraj(q_iniziale, q_alto, t);

% Debug
figure
subplot(2, 1, 1)
    plot(t, q_des', 'LineWidth', 2.0)
    grid on
    xlabel("Time [s]")
    ylabel("Joint Angles [rad]")
    title("Generated Trajectory in Config. Space")
    legend("q0", "q1", "q2", "q3", "q4", "q5")

subplot(2, 1, 2)
    plot(t, qdot_des', 'LineWidth', 2.0)
    grid on
    xlabel("Time [s]")
    ylabel("Derivative Joint Angles [rad/s]")
    title("Generated Trajectory in Config. Space")
    legend("qdot0", "qdot1", "qdot2", "qdot3", "qdot4", "qdot5")

%% Convert generated trajectory in Task Space
% Init Position and Orientation
p_des = zeros(length(t), 3);    % Position [m]
eul_des = zeros(length(t), 3);    % Euler RPY [rad]

for i = 1:length(t)
    T = Rob.fkine(q_des(i, :));
    p_des(i, :) = T.t';
    eul_des(i, :) = rotm2eul(T.R);
end

figure
subplot(2, 1, 1)
    plot(t, p_des', 'LineWidth', 2.0)
    grid on
    xlabel("Time [s]")
    ylabel("End-Effector Position [m]")
    title("Generated Trajectory in Task Space")
    legend("x", "y", "z")

subplot(2, 1, 2)
    plot(t, eul_des', 'LineWidth', 2.0)
    grid on
    xlabel("Time [s]")
    ylabel("End-Effector Quaternion")
    title("Generated Trajectory in Task Space")
    legend("roll", "pitch", "yaw")

%% Simulation
% Timeseries of the joint variables of the real robot
q = q0;
qdot = qdot0;

% % Start Cycling
% for i = 1:length(t)
% 
% end



%% Visualization
% % Init Figure
% figure;
% Rob.plot(q_iniziale); % Visualize init configuration
% 
% % Compute position of the initial marker
% T_iniziale = Rob.fkine(q_iniziale);
% hold on
% plot3(T_iniziale.t(1), T_iniziale.t(2), T_iniziale.t(3), 'g^', 'MarkerSize', 6, 'MarkerFaceColor', 'none');
% hold off