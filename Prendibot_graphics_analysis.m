%% Load Functions and Data folders
% In questa sezione, carico i dati calcolati dagli altri programmi e funzioni utili. %

addpath("functions")
addpath("data")
addpath("video")

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
hold off
grid on
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
legend("Desired Trajectory", "EE Position")
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

% Nota sul wrap2pi:
% Quando si esegue una differenza di angoli, potrebbe succedere che fai
% pi  - (- pi) = 2pi. L'errore tuttavia non è 0 anche se pi e -pi sono lo
% stesso numero, se si definiscono gli angoli nel range [-pi, pi].
% Se si immagina anche un controllore proporzionale alla differenza di
% errore, produrrebbe una reazione ad un errore di 2pi (che sarebbe anche
% molto alto) in una situazione in cui invece in realtà l'errore sarebbe
% nullo.

% Per ritraslare tutti gli angoli dentro il range di definizione, si usa
% solitamente fare:
% angle_error = atan2(sin(angle2 - angle1), cos(angle2 - angle1));

%% Functions
function wrapped_angle = wrap2pi(angle)
    wrapped_angle = atan2(sin(angle), cos(angle));
end

