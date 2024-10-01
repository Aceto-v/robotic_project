%%% Script visualizzazione grafici %%%

%% Load Functions and Data folders

addpath("functions")
addpath("data")
addpath("video")

% Workpoint
load('data\punti_workpoint.mat', 'pos_iniziale', 'p_alto', 'p_meta_altezza', 'p_terra');
% Inverse Kinematics Data
load('data\risultati_invKin_Prendibot.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibot_workspace.mat', 'Rob', 'workspace');
% Robot Simulation
load('data\prendibot_simulation_result', 't', 'p_des', 'p_robot', 'eul_des', 'eul_robot');

%% Plot Result
figure
plot(t, p_des - p_robot, 'LineWidth', 2.0)
grid on
xlabel("t [s]")
ylabel("EE position error [m]")
legend("x", "y", "z")
title("EE position error")

figure
plot(t, wrap2pi(eul_des - eul_robot), 'LineWidth', 2.0)
grid on
xlabel("t [s]")
ylabel("EE orientation error [rad]")
legend("roll", "pitch", "yaw")
title("EE orientation error")

figure
plot3(p_des(1, :),  p_des(2, :), p_des(3, :), 'LineWidth', 2.0, 'LineStyle', '--')
hold on
plot3(p_robot(1, :),  p_robot(2, :), p_robot(3, :), 'LineWidth', 2.0)
plot3(p_des(1, 1), p_des(2, 1), p_des(3, 1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % q_iniziale
plot3(p_des(1, round(length(t)/(n_configs - 1))), p_des(2, round(length(t)/(n_configs - 1))), p_des(3, round(length(t)/(n_configs - 1))), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % q_alto
plot3(p_des(1, 2*round(length(t)/(n_configs - 1))), p_des(2, 2*round(length(t)/(n_configs - 1))), p_des(3, 2*round(length(t)/(n_configs - 1))), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % q_meta_altezza
plot3(p_des(1, 3*round(length(t)/(n_configs - 1))), p_des(2, 3*round(length(t)/(n_configs - 1))), p_des(3, 3*round(length(t)/(n_configs - 1))), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % q_terra
hold off
grid on
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
legend("Desired Trajectory", "EE Position","Initial Position", "High/Middle/Ground Position")
title("Desired and Real Trajectory")

figure
plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Workspace del Braccio Robotico');
grid on;
legend("Punti del WS");
axis equal

%% Functions
function wrapped_angle = wrap2pi(angle)
    wrapped_angle = atan2(sin(angle), cos(angle));
end

