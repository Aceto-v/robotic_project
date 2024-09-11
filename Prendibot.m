% Creazione del file di simulazione
%% Load Functions and Data folders
addpath("functions")
addpath("data")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Definizione parametri di simulazione
% Time Settings
tf = 1e1;    % [s]
fs = 1e3;    % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-2;

% Init Condition
q0 = q_iniziale';
qdot0 = zeros(length(q0), 1);

%% Generazionee della Traiettoria

%% Simulazione

%% Visualizzazione
% figure;
% Rob.plot(q_iniziale); % Visualizzazione della configurazione iniziale
% 
% % Visualizzazione del workspace
% plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.', 'MarkerSize', 2); 
% hold on;
% 
% % Calcola la posizione del segnalino per la posizione iniziale
% T_iniziale = Rob.fkine(q_iniziale);
% plot3(T_iniziale.t(1), T_iniziale.t(2), T_iniziale.t(3), 'g^', 'MarkerSize', 6, 'MarkerFaceColor', 'none');

%% Grafico Traiettoria
% % Visualizzazione della traiettoria dell'endeffector nello spazio 3D
% figure;
% plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'g-', 'LineWidth', 2);
% xlabel('X  [cm]');
% ylabel('Y  [cm]');
% zlabel('Z  [cm]');
% title('Traiettoria dell''Endeffector nello spazio 3D');
% grid on;
% axis equal;
% % Aggiunta di marcatori per punti chiave della traiettoria
% hold on;
% plot3(traj_endeffector(1,1), traj_endeffector(1,2), traj_endeffector(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Punto iniziale
% plot3(traj_endeffector(end,1), traj_endeffector(end,2), traj_endeffector(end,3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Punto finale
% legend('Traiettoria', 'Punto Iniziale', 'Punto Finale');
% hold off;

%% Creazione file video
% % Creazione dell'oggetto video
% video_filename = 'Prendibot_simulation.mp4';
% v = VideoWriter(video_filename, 'MPEG-4');
% v.FrameRate = 15; % Impostare il frame rate desiderato
% open(v);
% % Chiudere il file video
% close(v);
% disp(['Video saved as ', video_filename]);

%% Salvataggio risultati
% save('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory', 't_total', 'dt');
% disp('Dati salvati in prendibot_trajectory_analysis.mat');


