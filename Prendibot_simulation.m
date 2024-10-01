%%% Script simulazione %%%

%% Load Functions and Data folders

addpath("functions")
addpath("data")
addpath("video")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibot.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibot_workspace.mat', 'Rob', 'workspace');

%% Define Simulation Parameters

% Configurations
config_sequence = {q_iniziale, q_alto, q_meta_altezza, q_terra, q_iniziale};
n_configs = length(config_sequence);

% Time Settings
tf = 1e1*(n_configs - 1);    % [s]
fs = 1e2;    % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-3;     % avoid numerical singularities

% Init Condition
q0 = q_iniziale';
% q0 = randn(length(q_iniziale), 1);
qdot0 = zeros(length(q0), 1);

%% Generate Trajectory

% Init variables
q_des = [];
qdot_des = [];
q2dot_des = [];

% Stuck the desired confs
for i = 1:(n_configs - 1)
    [q_act_des, qdot_act_des, q2dot_act_des] = jtraj(config_sequence{i}, config_sequence{i + 1}, t(1:round(length(t)/(n_configs - 1))));
    
    % Update
    q_des = [q_des; q_act_des];
    qdot_des = [qdot_des; qdot_act_des];
    q2dot_des = [q2dot_des; q2dot_act_des];
end
clear q_act_des qdot_act_des q2dot_act_des

q_des = [q_des; q_des(end, :)];
qdot_des = [qdot_des; qdot_des(end, :)];
q2dot_des = [q2dot_des; q2dot_des(end, :)];

% Traspongo perchè mi da fastidio a righe, mannaggia a chi a fatto jtraj
q_des = q_des';
qdot_des = qdot_des';
q2dot_des = q2dot_des';

%% Convert generated trajectory in Task Space

% Init Position and Orientation
p_des = zeros(3, length(t));    % Position [m]
eul_des = zeros(3, length(t));    % Euler RPY [rad]
xi_des = zeros(6, length(t));           % Generalized velocity vector

for i = 1:length(t)
    % Compute Forward and Differential Kinematics
    T = Rob.fkine(q_des(:, i));
    J = Rob.jacob0(q_des(:, i));
    % Ja = ;

    % Extract Pose (Position + Orientation)
    p_des(:, i) = T.t;
    eul_des(:, i) = rotm2eul(T.R);

    % Extract the Velocity Twist
    xi_des(:, i) = J*qdot_des(:, i);
end

%% Simulation
% Timeseries of the joint variables of the real robot
q = q0;
qdot = qdot0;
p_robot = Rob.fkine(q).t;
% K = 1e1;
K = 1e1;

% Definizione dell'oggetto VideoWriter
video_filename = 'video\Prendibot_simulation.mp4'; % Definizione del nome del file e il formato
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fs; % Impostazione del framerate al valore di campionamento fs
open(v);

figure;
Rob.plot(q_iniziale);
hold on;

% Start Cycling
for i = 1:(length(t) - 1)
    % Calcolo il jacobiano alla configurazione attuale
    J = Rob.jacob0(q(:, i));
    % Regolarizzazione
    J = J + epsilon*eye(size(J));
    
    % Select Position Jacobian
    Jp = J(1:3, :);
    % Position Error
    ep = p_des(:, i) - p_robot(:, i);
    qdot(:, i) = Jp \ (xi_des(1:3, i) + K*(ep));


    % Integration
    % q(k) = q(k - 1) + Ts*q_dot(k - 1)
    q(:, i + 1) = q(:, i) + (1/fs)*qdot(:, i);

    %Update Robot Position
    p_robot(:, i + 1) = Rob.fkine(q(:, i + 1)).t;
    eul_robot(:, i + 1) = rotm2eul(Rob.fkine(q(:, i + 1)).R);

    %Plot aggiornato per ogni passo
    if mod(i, 15) == 0
        figure(1); % Use a specific figure for the robot plot
        Rob.plot(q(:, i)');
        hold on;
        plot3(p_robot(1, 1:i), p_robot(2, 1:i), p_robot(3, 1:i), 'r-', 'LineWidth', 2);
        plot3(p_des(1, 1), p_des(2, 1), p_des(3, 1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % posizione iniziale
        hold off;
        
        % Capture the current frame for the video
        frame = getframe(gcf);  
        writeVideo(v, frame);
        pause(0.1);
    end

end

% chiusura video
close(v);
disp(['Video saved as ', video_filename]);

%% Save Result
% Salva le variabili necessarie per i grafici
save('data\prendibot_simulation_result', 't', 'p_des', 'p_robot', 'eul_des', 'eul_robot', 'workspace');
