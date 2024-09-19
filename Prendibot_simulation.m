%% Progetto Robotica Industriale - Prendibotv12
% Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Script simulazione delle traiettorie

%% Load Functions and Data folders
addpath("functions")
addpath("data")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Definizione Parametri Simulazione
% Time Settings
tf = 1e1;    % [s]
fs = 1e3;    % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-2;

% Init Condition
q0 = q_iniziale';
qdot0 = zeros(length(q0), 1);

%% Generazione Traiettoria

%% Simulazione

%% Grafico Traiettoria
% Visualizzazione della traiettoria dell'endeffector nello spazio 3D
figure;
plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'g-', 'LineWidth', 2);
xlabel('X  [cm]');
ylabel('Y  [cm]');
zlabel('Z  [cm]');
title('Traiettoria dell''Endeffector nello spazio 3D');
grid on;
axis equal;
% Aggiunta di marcatori per punti chiave della traiettoria
hold on;
% Punto Iniziale
plot3(traj_endeffector(1,1), traj_endeffector(1,2), traj_endeffector(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% Punto Finale
plot3(traj_endeffector(end,1), traj_endeffector(end,2), traj_endeffector(end,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
% Punto Alto
T_alto = Rob.fkine(q_alto);
plot3(T_alto.t(1), T_alto.t(2), T_alto.t(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
% Punto Meta Altezza
T_meta_altezza = Rob.fkine(q_meta_altezza);
plot3(T_meta_altezza.t(1), T_meta_altezza.t(2), T_meta_altezza.t(3), 'co', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
% Punto Terra
T_terra = Rob.fkine(q_terra);
plot3(T_terra.t(1), T_terra.t(2), T_terra.t(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

legend('Traiettoria', 'Punto Iniziale', 'Punto Finale', 'Punto Alto', 'Punto Metà Altezza', 'Punto Terra');
hold off;

%% Salvataggio Risultati
% Salva i risultati dell'analisi
save('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory', 't_total', 'dt');
disp('Dati salvati in prendibot_trajectory_analysis.mat');