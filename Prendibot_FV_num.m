% Progetto Robotica Industriale - Prendibotv12
% Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Script simulazione delle traiettorie

%% Current Version
% Carica i risultati della cinematica inversa
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Carica il robot e il workspace
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

% Parametri del controllo del movimento
num_steps = 100;
dt = 0.1; % Intervallo di tempo
epsilon = 0.01; % Fattore di regolarizzazione

% Creazione dell'oggetto video
video_filename = 'Prendibot_simulation_fv.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 15; % Impostare il frame rate desiderato
open(v);

% Inizializza la figura per la visualizzazione
figure;
Rob.plot(q_iniziale); % Visualizzazione della configurazione iniziale

% Visualizzazione del workspace (new)
plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.', 'MarkerSize', 2); 
hold on;

% Calcola la posizione del segnalino per la posizione iniziale
T_iniziale = Rob.fkine(q_iniziale);
plot3(T_iniziale.t(1), T_iniziale.t(2), T_iniziale.t(3), 'g^', 'MarkerSize', 6, 'MarkerFaceColor', 'none');

% Array per memorizzare le posizioni dell'endeffector
traj_endeffector = [];
errors_position = [];
errors_orientation = [];
joint_velocities = [];
joint_accelerations = [];
singularity_values = [];
q_trajectory = [];

% Esegui le traiettorie tra le diverse posizioni

% Iniziale -> Alto
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_iniziale, q_alto, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% Alto -> Metà Altezza
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_alto, q_meta_altezza, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% Metà Altezza -> Terra
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_meta_altezza, q_terra, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);
% Terra -> Iniziale
[traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = ...
    exc_traj_jacobian_analysis(Rob, q_terra, q_iniziale, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v);

% Chiudere il file video
close(v);
disp(['Video saved as ', video_filename]);

% Visualizzazione della traiettoria dell'endeffector nello spazio 3D
figure;
plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'g-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Traiettoria dell''Endeffector nello spazio 3D');
grid on;
axis equal;
% Aggiunta di marcatori per punti chiave della traiettoria
hold on;
plot3(traj_endeffector(1,1), traj_endeffector(1,2), traj_endeffector(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Punto iniziale
plot3(traj_endeffector(end,1), traj_endeffector(end,2), traj_endeffector(end,3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Punto finale
legend('Traiettoria', 'Punto Iniziale', 'Punto Finale');
hold off;

% Salva i risultati dell'analisi
save('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory');

disp('Dati salvati in prendibot_trajectory_analysis.mat');

%% Chaos Version
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

