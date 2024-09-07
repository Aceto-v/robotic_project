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

%% Failur idea
% % Analisi dell'occupazione del workspace(new)
% threshold = 0.1; % Soglia di tolleranza
% occupazione_workspace = sum(min(pdist2(traj_endeffector, workspace), [], 2) < threshold) / size(workspace, 1);
% disp(['Percentuale del workspace esplorata: ', num2str(occupazione_workspace*100), '%']);

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

%% OLD (Vicio's Fuction)
% function [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = esegui_traiettoria_jacobian_analisi(Rob, q_start, q_end, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v)
%     traj = jtraj(q_start, q_end, num_steps);
% 
%     for i = 1:num_steps
%         % Posizione attuale
%         q = traj(i, :);
% 
%         % Calcolo dello Jacobiano
%         J = Rob.jacob0(q);
% 
%         % Calcolo della posizione desiderata e attuale dell'end-effector
%         T_desired = Rob.fkine(traj(i, :));
%         T_current = Rob.fkine(q);
% 
%         % Calcolo della velocità desiderata dell'end-effector
%         dx_pos = (T_desired.t - T_current.t) / dt;
% 
%         % Converti le matrici di rotazione in quaternioni per una differenza più accurata
%         R_desired = T_desired.R;
%         R_current = T_current.R;
% 
%         % Calcolo della differenza di orientamento
%         dR = R_desired * R_current';
%         dtheta = tr2rpy(dR)'; % Assicurarsi che sia un vettore colonna
% 
%         % Combinazione delle velocità di posizione e orientamento
%         dx = [dx_pos; dtheta];
% 
%         % Aggiunta della regolarizzazione per evitare problemi di singolarità
%         J_inv = pinv(J + epsilon * eye(size(J)));
% 
%         % Calcolo della variazione delle coordinate articolari
%         dq = J_inv * dx;
% 
%         % Aggiornamento della posizione articolare
%         q = q + (dq' * dt);
% 
%         % Applicazione dei limiti delle giunture
%         q = max(Rob.qlim(:,1)', min(Rob.qlim(:,2)', q));
% 
%         % Memorizzazione della posizione dell'endeffector
%         T = Rob.fkine(q);
%         traj_endeffector = [traj_endeffector; T.t'];
% 
%         % Calcolo degli errori di posizione e orientamento
%         T_current = Rob.fkine(q);
%         pos_error = norm(T_current.t' - traj(i, 1:3));
%         orient_error = norm(tr2rpy(T_current.R) - tr2rpy(Rob.fkine(traj(i, :)).R));
%         errors_position = [errors_position; pos_error];
%         errors_orientation = [errors_orientation; orient_error];
% 
%         % Calcolo della velocità dei giunti
%         joint_vel = norm(dq);
%         joint_velocities = [joint_velocities; joint_vel];
% 
%         % Calcolo dell'accelerazione dei giunti
%         if i > 1
%             joint_acc = (joint_vel - joint_velocities(end-1)) / dt;
%             joint_accelerations = [joint_accelerations; joint_acc];
%         else
%             joint_accelerations = [joint_accelerations; 0];
%         end
% 
%         % Calcolo dei valori singolari del Jacobiano
%         singular_val = svd(J);
%         singularity_values = [singularity_values; singular_val'];
% 
%         % Memorizzazione della traiettoria articolare
%         q_trajectory = [q_trajectory; q];
% 
%         % Simulazione del movimento del robot
%         if mod(i, 5) == 0 
%             Rob.plot(q);
%             plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'r-', 'LineWidth', 2);
%             frame = getframe(gcf);
%             writeVideo(v, frame); % Scrive il frame nel video
%             pause(0.1); % Aggiunta di una pausa per la visualizzazione
%         end
%     end
% 
%     % Visualizza i segnalini per le posizioni chiave
%     plot3(T.t(1), T.t(2), T.t(3), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'none');
% 
% 
%     % Verifica se la posizione finale raggiunta è quella desiderata
%     disp('Posizione finale raggiunta:');
%     disp(q);
%     disp('Posizione finale desiderata:');
%     disp(q_end);
% end

%% NEW Function (Dani's version)
% function [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = esegui_traiettoria_jacobian_analisi(Rob, q_start, q_end, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v)
%     %% Trajectory Generation
%     [q_des, qdot_des, q2dot_des] = jtraj(q_start, q_end, num_steps);
% 
%     % Gain for the Inverse Kinematics controller
%     clik_gain = 1e2*eye(size(q_des, 2));
% 
%     %% Cycle over simulation time
%     for i = 1:num_steps
%         % Compute Jacobian of the desired configuration (q_des)
%         J_des = Rob.jacob0(q_des(i, :));
% 
%         % Desired End-Effector Velocity Twist (i.e., linear velocity;
%         % angular velocity)
%         % Here, we're using: 
%         % velocity_twist = Jacobian*qdot
%         ee_des_vel = J_des*qdot_des(i, :);
% 
%         %% Inverse Kinematics (Open-Loop)
%         J_inv = pinv(J + epsilon * eye(size(J)));
% 
%         % Commanded joint velocities to the simulated robot
%         error = ee_des_vel - ;
%         q_dot = J_inv*(ee_des_vel + clik_gain*());
%     end
% 
%     for i = 1:num_steps
%         % Posizione attuale
%         q = traj(i, :);
% 
%         % Calcolo dello Jacobiano
%         J = Rob.jacob0(q);
% 
%         % Calcolo della posizione desiderata e attuale dell'end-effector
%         T_desired = Rob.fkine(traj(i, :));
%         T_current = Rob.fkine(q);
% 
%         % Calcolo della velocità desiderata dell'end-effector
%         dx_pos = (T_desired.t - T_current.t) / dt;
% 
%         % Converti le matrici di rotazione in quaternioni per una differenza più accurata
%         R_desired = T_desired.R;
%         R_current = T_current.R;
% 
%         % Calcolo della differenza di orientamento
%         dR = R_desired * R_current';
%         dtheta = tr2rpy(dR)'; % Assicurarsi che sia un vettore colonna
% 
%         % Combinazione delle velocità di posizione e orientamento
%         dx = [dx_pos; dtheta];
% 
%         % Aggiunta della regolarizzazione per evitare problemi di singolarità
%         J_inv = pinv(J + epsilon * eye(size(J)));
% 
%         % Calcolo della variazione delle coordinate articolari
%         dq = J_inv * dx;
% 
%         % Aggiornamento della posizione articolare
%         q = q + (dq' * dt);
% 
%         % Applicazione dei limiti delle giunture
%         q = max(Rob.qlim(:,1)', min(Rob.qlim(:,2)', q));
% 
%         % Memorizzazione della posizione dell'endeffector
%         T = Rob.fkine(q);
%         traj_endeffector = [traj_endeffector; T.t'];
% 
%         % Calcolo degli errori di posizione e orientamento
%         T_current = Rob.fkine(q);
%         pos_error = norm(T_current.t' - traj(i, 1:3));
%         orient_error = norm(tr2rpy(T_current.R) - tr2rpy(Rob.fkine(traj(i, :)).R));
%         errors_position = [errors_position; pos_error];
%         errors_orientation = [errors_orientation; orient_error];
% 
%         % Calcolo della velocità dei giunti
%         joint_vel = norm(dq);
%         joint_velocities = [joint_velocities; joint_vel];
% 
%         % Calcolo dell'accelerazione dei giunti
%         if i > 1
%             joint_acc = (joint_vel - joint_velocities(end-1)) / dt;
%             joint_accelerations = [joint_accelerations; joint_acc];
%         else
%             joint_accelerations = [joint_accelerations; 0];
%         end
% 
%         % Calcolo dei valori singolari del Jacobiano
%         singular_val = svd(J);
%         singularity_values = [singularity_values; singular_val'];
% 
%         % Memorizzazione della traiettoria articolare
%         q_trajectory = [q_trajectory; q];
% 
%         % Simulazione del movimento del robot
%         if mod(i, 5) == 0 
%             Rob.plot(q);
%             plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'r-', 'LineWidth', 2);
%             frame = getframe(gcf);
%             writeVideo(v, frame); % Scrive il frame nel video
%             pause(0.1); % Aggiunta di una pausa per la visualizzazione
%         end
%     end
% 
%     % Visualizza i segnalini per le posizioni chiave
%     plot3(T.t(1), T.t(2), T.t(3), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'none');
% 
%     % Verifica se la posizione finale raggiunta è quella desiderata
%     disp('Posizione finale raggiunta:');
%     disp(q);
%     disp('Posizione finale desiderata:');
%     disp(q_end);
% end




