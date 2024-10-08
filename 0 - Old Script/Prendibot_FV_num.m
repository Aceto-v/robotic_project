%% Progetto Robotica Industriale - Prendibotv12
% Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Script simulazione delle traiettorie

%% Caricamento cartella Functions and Data
addpath("data")

% Carica i risultati della cinematica inversa
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Carica il robot e il workspace
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Definizione parametri di simulazione
t_total = 100; % Tempo totale della simulazione in secondi
dt = 0.2; % Intervallo di tempo
t = 0;
epsilon = 1e-2; % Fattore di regolarizzazione

%% Inizializzazione simulazione
% Creazione dell'oggetto video
video_filename = 'Prendibot_simulation_fv.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 15; % Impostare il frame rate desiderato
open(v);

% Inizializza la figura per la visualizzazione
figure;
Rob.plot(q_iniziale); % Visualizzazione della configurazione iniziale

% Visualizzazione del workspace
plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.', 'MarkerSize', 1);
hold on;

% Calcola la posizione del segnalino per la posizione iniziale
T_iniziale = Rob.fkine(q_iniziale);
plot3(T_iniziale.t(1), T_iniziale.t(2), T_iniziale.t(3), 'go', 'MarkerSize', 7, 'MarkerFaceColor', '#7E2F8E');

%% Generazione traiettoria
% Array per memorizzare le posizioni dell'endeffector
traj_endeffector = [];
errors_position = [];
errors_orientation = [];
joint_velocities = [];
joint_accelerations = [];
singularity_values = [];
q_trajectory = [];

% Esegui le traiettorie tra le diverse posizioni (t_total)

% Iniziale -> Alto
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_iniziale, q_alto, t_total, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);

% Alto -> Metà Altezza
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_alto, q_meta_altezza, t_total, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);

% Metà Altezza -> Terra
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_meta_altezza, q_terra, t_total, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);

% Terra -> Iniziale
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_terra, q_iniziale, t_total, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);

% Chiudere il file video
close(v);
disp(['Video saved as ', video_filename]);

%% Grafico traiettoria
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

%% Salvataggio risultati
% Salva i risultati dell'analisi
save('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory', 't_total', 'dt');

disp('Dati salvati in prendibot_trajectory_analysis.mat');

%% Old Version
% % Script simulazione delle traiettorie (modificato)
% 
% % Carica i risultati della cinematica inversa automatica
% load('data\risultati_invKin_Prendibotv12.mat', 'q_positions', 'selected_positions');
% 
% % Carica il robot e il workspace
% load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');
% 
% % Parametri del controllo del movimento
% num_steps = 100;
% dt = 0.1; % Intervallo di tempo
% epsilon = 0.01; % Fattore di regolarizzazione
% 
% % Creazione dell'oggetto video
% video_filename = 'Prendibot_simulation.mp4';
% v = VideoWriter(video_filename, 'MPEG-4');
% v.FrameRate = 15; % Impostare il frame rate desiderato
% open(v);
% 
% % Inizializza la figura per la visualizzazione
% figure;
% Rob.plot(q_positions(1, :)); % Visualizzazione della configurazione iniziale
% 
% % Visualizzazione del workspace
% plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.', 'MarkerSize', 2); 
% hold on;
% 
% % Calcola la posizione del segnalino per la posizione iniziale
% T_iniziale = Rob.fkine(q_positions(1, :));
% plot3(T_iniziale.t(1), T_iniziale.t(2), T_iniziale.t(3), 'g^', 'MarkerSize', 6, 'MarkerFaceColor', 'none');
% 
% % Memorizza il manico del grafico corrente
% hold on;
% 
% % Array per memorizzare le posizioni dell'endeffector
% traj_endeffector = [];
% errors_position = [];
% errors_orientation = [];
% joint_velocities = [];
% joint_accelerations = [];
% singularity_values = [];
% q_trajectory = [];
% 
% % Esegui le traiettorie tra le diverse posizioni
% % Da posizione 1 a posizione 2
% [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
%     esegui_traiettoria_jacobian_analisi(Rob, q_positions(1, :), q_positions(2, :), num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% % Da posizione 2 a posizione 3
% [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
%     esegui_traiettoria_jacobian_analisi(Rob, q_positions(2, :), q_positions(3, :), num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% % Da posizione 3 a posizione 4
% [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
%     esegui_traiettoria_jacobian_analisi(Rob, q_positions(3, :), q_positions(4, :), num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% % Da posizione 4 a posizione 1 (ritorno alla configurazione iniziale)
% [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
%     esegui_traiettoria_jacobian_analisi(Rob, q_positions(4, :), q_positions(1, :), num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% 
% % Chiudere il file video
% hold off;
% close(v);
% disp(['Video saved as ', video_filename]);
% 
% % Visualizzazione della traiettoria dell'endeffector nello spazio 3D
% figure;
% plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'g-', 'LineWidth', 2);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Traiettoria dell''Endeffector nello spazio 3D');
% grid on;
% axis equal;
% 
% % Aggiunta di marcatori per punti chiave della traiettoria
% hold on;
% plot3(traj_endeffector(1,1), traj_endeffector(1,2), traj_endeffector(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Punto iniziale
% plot3(traj_endeffector(end,1), traj_endeffector(end,2), traj_endeffector(end,3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Punto finale
% legend('Traiettoria', 'Punto Iniziale', 'Punto Finale');
% hold off;
% 
% % Analisi dell'occupazione del workspace
% threshold = 0.1; % Soglia di tolleranza
% occupazione_workspace = sum(min(pdist2(traj_endeffector, workspace), [], 2) < threshold) / size(workspace, 1);
% disp(['Percentuale del workspace esplorata: ', num2str(occupazione_workspace*100), '%']);
% 
% % Salva i risultati dell'analisi
% save('data\prendibot_trajectory_analysis_auto.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory');

