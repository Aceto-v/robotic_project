%% Progetto Robotica Industriale - Prendibotv12
% Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra

% Script simulazione e visualizzazione delle traiettorie del manipolatore

%% Load Functions and Data folders
addpath("functions")
addpath("data")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Definizione Parametri Simulazione
% Time Settings
tf = 10;          % [s]
fs = 500;         % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-2;

% Init Condition
q0 = q_iniziale';
qdot0 = zeros(length(q0), 1);

%% Generazione Traiettoria
% Genera le traiettorie tra i punti chiave e concatena
[q1, qdot1, q2dot1] = jtraj(q_iniziale, q_alto, length(t));
[q2, qdot2, q2dot2] = jtraj(q_alto, q_meta_altezza, length(t));
[q3, qdot3, q2dot3] = jtraj(q_meta_altezza, q_terra, length(t));
[q4, qdot4, q2dot4] = jtraj(q_terra, q_iniziale, length(t));

% Concatenazione delle traiettorie
q_des = [q1; q2; q3; q4];
qdot_des = [qdot1; qdot2; qdot3; qdot4];
q2dot_des = [q2dot1; q2dot2; q2dot3; q2dot4];
t_total = (0:(1/fs):(tf*4))';

n_steps = length(q_des);

%% Calcolo della Traiettoria Desiderata dell'End-Effector
% Preallocazione per efficienza
traj_endeffector = zeros(n_steps, 3);
for j = 1:n_steps
    T = Rob.fkine(q_des(j, :));
    traj_endeffector(j, :) = T.t(1:3)';
end

%% Simulazione
% Inizializzazione delle variabili
q = q0;
qdot = qdot0;

% Preallocazione per efficienza
q_actual = zeros(length(q0), n_steps);
qdot_actual = zeros(length(q0), n_steps);
ee_actual = zeros(n_steps, 3);
ee_desired = traj_endeffector; % Dato che abbiamo già calcolato la traiettoria desiderata
e_position = zeros(n_steps, 3);
e_orientation = zeros(n_steps, 3);
q_eul_actual = zeros(n_steps, 3);
q_quat_actual = zeros(n_steps, 4);

% Parametri del controllo
gain = 1; % Guadagno del controllore, può essere regolato

% Inizializzazione Video Writer
video_filename = 'robot_trajectory.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 5;
open(v);

%% Inizializzazione Figura per la Simulazione e la Traiettoria 3D
figure('Name', 'Simulazione e Traiettoria del Manipolatore', 'NumberTitle', 'off');
hold on;

% Plot della Traiettoria Desiderata
plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Traiettoria Desiderata');

% Plot dei Punti Chiave
% Punto Iniziale
plot3(traj_endeffector(1,1), traj_endeffector(1,2), traj_endeffector(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Punto Iniziale');
% Punto Finale (uguale al punto iniziale in questo caso)
plot3(traj_endeffector(end,1), traj_endeffector(end,2), traj_endeffector(end,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', 'Punto Finale');
% Punto Alto
T_alto = Rob.fkine(q_alto);
plot3(T_alto.t(1), T_alto.t(2), T_alto.t(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm', 'DisplayName', 'Punto Alto');
% Punto Meta Altezza
T_meta_altezza = Rob.fkine(q_meta_altezza);
plot3(T_meta_altezza.t(1), T_meta_altezza.t(2), T_meta_altezza.t(3), 'co', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'DisplayName', 'Punto Metà Altezza');
% Punto Terra
T_terra = Rob.fkine(q_terra);
plot3(T_terra.t(1), T_terra.t(2), T_terra.t(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'DisplayName', 'Punto Terra');

% Inizializzazione della Traiettoria Effettiva con AnimatedLine
actual_traj = animatedline('Color', 'r', 'LineWidth', 2, 'DisplayName', 'Traiettoria Effettiva');

% Impostazioni della Figura
xlabel('X [cm]');
ylabel('Y [cm]');
zlabel('Z [cm]');
title('Simulazione del Movimento del Manipolatore con Traiettoria 3D');
grid on;
axis equal;
legend('show');
view(3); % Vista 3D

% Inizializzazione del Plot del Robot
Rob.plot(q', 'workspace', workspace, 'delay', 0, 'nowrist', 'noname');

% Cattura del primo frame
frame = getframe(gcf);
writeVideo(v, frame); % Scrive il frame nel video

%% Start Cycling
for i = 1:n_steps
    % Traiettoria desiderata al passo corrente
    qd = q_des(i,:)';
    qd_dot = qdot_des(i,:)';
    qd_ddot = q2dot_des(i,:)';
    
    % Calcolo della posa desiderata dell'endeffector
    T_des = Rob.fkine(qd');
    pos_des = T_des.t(1:3);
    R_des = T_des.R;
    
    % Velocità desiderata dell'endeffector
    J_des = Rob.jacob0(qd');
    ee_des_vel = J_des * qd_dot;
    
    % Calcolo della posa attuale dell'endeffector
    T_actual = Rob.fkine(q');
    pos_actual = T_actual.t(1:3);
    R_actual = T_actual.R;
    
    % Calcolo degli errori di posizione
    e_pos = pos_des - pos_actual;
    
    % Calcolo degli errori di orientamento utilizzando RPY
    rpy_des = rotm2eul(R_des, 'XYZ');
    rpy_actual = rotm2eul(R_actual, 'XYZ');
    e_orient = rpy_des - rpy_actual;
    
    % Memorizzazione degli errori
    e_position(i,:) = e_pos';
    e_orientation(i,:) = e_orient';
    
    % Definizione del vettore di errore completo (posizione e orientamento)
    error = [e_pos; e_orient'];
    
    % Calcolo della velocità dei giunti utilizzando il controllore CLIK
    u = clik(Rob, q, error, ee_des_vel, gain);
    
    % Aggiornamento delle velocità e posizioni dei giunti
    qdot = u;
    q = q + qdot / fs;
    
    % Memorizzazione dei valori attuali
    q_actual(:,i) = q;
    qdot_actual(:,i) = qdot;
    ee_actual(i,:) = pos_actual';
    
    % Calcolo degli angoli di Eulero e dei quaternioni
    q_eul_actual(i,:) = rpy_actual;
    q_quat_actual(i,:) = rotm2quat(R_actual);
    
    % Aggiornamento della visualizzazione del robot
    Rob.plot(q', 'workspace', workspace, 'delay', 0, 'nowrist', 'noname');
    
    % Aggiornamento della Traiettoria Effettiva
    addpoints(actual_traj, ee_actual(i,1), ee_actual(i,2), ee_actual(i,3));
    drawnow limitrate; % Aggiorna la figura
    
    % Cattura del frame per il video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Chiusura del Video Writer
close(v);
disp(['Video salvato come ', video_filename]);

%% Salvataggio dei Dati della Simulazione
save('data\simulazione_traiettoria.mat', ...
    'q_actual', 'qdot_actual', 'q2dot_des', ...
    'ee_actual', 'ee_desired', 'e_position', 'e_orientation', ...
    'q_eul_actual', 'q_quat_actual', 't_total');

%% Grafico Traiettoria e Altri Dati
% Creazione di una nuova figura per i grafici
figure;

% Sottografico 1: Posizioni dei giunti
subplot(3,2,1);
plot(t_total, q_actual');
xlabel('Tempo [s]');
ylabel('Angolo [rad]');
title('Posizioni dei Giunti');
legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
grid on;

% Sottografico 2: Velocità dei giunti
subplot(3,2,2);
plot(t_total, qdot_actual');
xlabel('Tempo [s]');
ylabel('Velocità [rad/s]');
title('Velocità dei Giunti');
legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
grid on;

% Sottografico 3: Accelerazioni dei giunti
subplot(3,2,3);
plot(t_total, q2dot_des');
xlabel('Tempo [s]');
ylabel('Accelerazione [rad/s^2]');
title('Accelerazioni dei Giunti');
legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
grid on;

% Sottografico 4: Traiettoria dell'End-Effector
subplot(3,2,4);
plot3(ee_actual(:,1), ee_actual(:,2), ee_actual(:,3), 'b', ...
      ee_desired(:,1), ee_desired(:,2), ee_desired(:,3), 'r--');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Traiettoria dell''End-Effector');
legend('Reale', 'Desiderata');
grid on;
axis equal;

% Sottografico 5: Errori di Posizione
subplot(3,2,5);
plot(t_total, e_position');
xlabel('Tempo [s]');
ylabel('Errore di Posizione [m]');
title('Errore di Posizione dell''End-Effector');
legend('e_x', 'e_y', 'e_z');
grid on;

% Sottografico 6: Errori di Orientamento
subplot(3,2,6);
plot(t_total, e_orientation');
xlabel('Tempo [s]');
ylabel('Errore di Orientamento [rad]');
title('Errore di Orientamento dell''End-Effector');
legend('e_{roll}', 'e_{pitch}', 'e_{yaw}');
grid on;

%% Grafico Angoli di Eulero e Quaternioni
figure;

% Sottografico 1: Angoli di Eulero
subplot(2,1,1);
plot(t_total, q_eul_actual');
xlabel('Tempo [s]');
ylabel('Angoli di Eulero [rad]');
title('Angoli di Eulero (RPY) dell''End-Effector');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% Sottografico 2: Quaternioni
subplot(2,1,2);
plot(t_total, q_quat_actual');
xlabel('Tempo [s]');
ylabel('Quaternioni');
title('Quaternioni dell''End-Effector');
legend('q_1', 'q_2', 'q_3', 'q_4');
grid on;


% % Script simulazione delle traiettorie (Versione 1)
% 
% %% Load Functions and Data folders
% 
% addpath("functions")
% addpath("data")
% 
% % Inverse Kinematics Data
% load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% % Robot object and Workspace Analysis
% load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');
% 
% %% Definizione Parametri Simulazione
% 
% % Time Settings
% tf = 1e1;    % [s]
% fs = 1e3;    % [Hz]
% t = (0:(1/fs):tf)';
% 
% % Regularization
% epsilon = 1e-2;
% 
% % Init Condition
% q0 = q_iniziale';
% qdot0 = zeros(length(q0), 1);
% 
% %% Generazione Traiettoria
% 
% % Genera le traiettorie tra i punti chiave e concatena
% [q1, qdot1, q2dot1] = jtraj(q_iniziale, q_alto, t);
% [q2, qdot2, q2dot2] = jtraj(q_alto, q_meta_altezza, t);
% [q3, qdot3, q2dot3] = jtraj(q_meta_altezza, q_terra, t);
% [q4, qdot4, q2dot4] = jtraj(q_terra, q_iniziale, t);
% 
% % Concatenazione delle traiettorie
% q_des = [q1; q2; q3; q4];
% qdot_des = [qdot1; qdot2; qdot3; qdot4];
% q2dot_des = [q2dot1; q2dot2; q2dot3; q2dot4];
% t_total = (0:(1/fs):(tf*4))';
% 
% %% Simulazione
% 
% % Inizializzazione delle variabili
% n_steps = length(q_des);
% q = q0;
% qdot = qdot0;
% 
% % Preallocazione per efficienza
% q_actual = zeros(length(q0), n_steps);
% qdot_actual = zeros(length(q0), n_steps);
% ee_actual = zeros(n_steps, 3);
% ee_desired = zeros(n_steps, 3);
% e_position = zeros(n_steps, 3);
% e_orientation = zeros(n_steps, 3);
% q_eul_actual = zeros(n_steps, 3);
% q_quat_actual = zeros(n_steps, 4);
% 
% % Parametri del controllo
% gain = 1; % Guadagno del controllore, può essere regolato
% 
% % Inizializzazione Video Writer
% video_filename = 'robot_trajectory.mp4';
% v = VideoWriter(video_filename, 'MPEG-4');
% v.FrameRate = 15;
% open(v);
% 
% % Inizializzazione figura per la simulazione
% figure;
% hold on;
% Rob.plot(q','workspace', workspace);
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% title('Simulazione Traiettoria del Manipolatore');
% grid on;
% axis equal;
% 
% % Start Cycling
% for i = 1:n_steps
%     % Traiettoria desiderata al passo corrente
%     qd = q_des(i,:)';
%     qd_dot = qdot_des(i,:)';
%     qd_ddot = q2dot_des(i,:)';
% 
%     % Calcolo della posa desiderata dell'endeffector
%     T_des = Rob.fkine(qd');
%     pos_des = T_des.t;
%     R_des = T_des.R;
% 
%     % Velocità desiderata dell'endeffector
%     J_des = Rob.jacob0(qd');
%     ee_des_vel = J_des * qd_dot;
% 
%     % Calcolo della posa attuale dell'endeffector
%     T_actual = Rob.fkine(q');
%     pos_actual = T_actual.t;
%     R_actual = T_actual.R;
% 
%     % Calcolo degli errori di posizione
%     e_pos = pos_des - pos_actual;
% 
%     % Calcolo degli errori di orientamento utilizzando RPY
%     rpy_des = rotm2eul(R_des, 'XYZ');
%     rpy_actual = rotm2eul(R_actual, 'XYZ');
%     e_orient = rpy_des - rpy_actual;
% 
%     % Memorizzazione degli errori
%     e_position(i,:) = e_pos';
%     e_orientation(i,:) = e_orient';
% 
%     % Definizione del vettore di errore completo (posizione e orientamento)
%     error = [e_pos; e_orient'];
% 
%     % Calcolo della velocità dei giunti utilizzando il controllore CLIK
%     u = clik(Rob, q, error, ee_des_vel, gain);
% 
%     % Aggiornamento delle velocità e posizioni dei giunti
%     qdot = u;
%     q = q + qdot / fs;
% 
%     % Memorizzazione dei valori attuali
%     q_actual(:,i) = q;
%     qdot_actual(:,i) = qdot;
%     ee_actual(i,:) = pos_actual';
%     ee_desired(i,:) = pos_des';
% 
%     % Calcolo degli angoli di Eulero e dei quaternioni
%     q_eul_actual(i,:) = rpy_actual;
%     q_quat_actual(i,:) = rotm2quat(R_actual);
% 
%     % Aggiornamento della visualizzazione del robot
%     Rob.plot(q','workspace', workspace);
%     drawnow;
% 
%     % Cattura del frame per il video
%     frame = getframe(gcf);
%     writeVideo(v, frame);
% end
% 
% % Chiusura del Video Writer
% close(v);
% disp(['Video saved as ', video_filename]);
% 
% %% Grafico Traiettoria
% 
% % Creazione di una nuova figura per i grafici
% figure;
% 
% % Sottografico 1: Posizioni dei giunti
% subplot(3,2,1);
% plot(t_total, q_actual');
% xlabel('Tempo [s]');
% ylabel('Angolo [rad]');
% title('Posizioni dei Giunti');
% legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
% grid on;
% 
% % Sottografico 2: Velocità dei giunti
% subplot(3,2,2);
% plot(t_total, qdot_actual');
% xlabel('Tempo [s]');
% ylabel('Velocità [rad/s]');
% title('Velocità dei Giunti');
% legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
% grid on;
% 
% % Sottografico 3: Accelerazioni dei giunti
% subplot(3,2,3);
% plot(t_total, q2dot_des');
% xlabel('Tempo [s]');
% ylabel('Accelerazione [rad/s^2]');
% title('Accelerazioni dei Giunti');
% legend(arrayfun(@(x) sprintf('q%d',x), 1:length(q0), 'UniformOutput', false));
% grid on;
% 
% % Sottografico 4: Traiettoria dell'End-Effector
% subplot(3,2,4);
% plot3(ee_actual(:,1), ee_actual(:,2), ee_actual(:,3), 'b', ...
%       ee_desired(:,1), ee_desired(:,2), ee_desired(:,3), 'r--');
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% title('Traiettoria dell''End-Effector');
% legend('Reale', 'Desiderata');
% grid on;
% axis equal;
% 
% % Sottografico 5: Errori di Posizione
% subplot(3,2,5);
% plot(t_total, e_position');
% xlabel('Tempo [s]');
% ylabel('Errore di Posizione [m]');
% title('Errore di Posizione dell''End-Effector');
% legend('e_x', 'e_y', 'e_z');
% grid on;
% 
% % Sottografico 6: Errori di Orientamento
% subplot(3,2,6);
% plot(t_total, e_orientation');
% xlabel('Tempo [s]');
% ylabel('Errore di Orientamento [rad]');
% title('Errore di Orientamento dell''End-Effector');
% legend('e_{roll}', 'e_{pitch}', 'e_{yaw}');
% grid on;
% 
% % Creazione di una nuova figura per angoli di Eulero e quaternioni
% figure;
% 
% % Sottografico 1: Angoli di Eulero
% subplot(2,1,1);
% plot(t_total, q_eul_actual');
% xlabel('Tempo [s]');
% ylabel('Angoli di Eulero [rad]');
% title('Angoli di Eulero (RPY) dell''End-Effector');
% legend('Roll', 'Pitch', 'Yaw');
% grid on;
% 
% % Sottografico 2: Quaternioni
% subplot(2,1,2);
% plot(t_total, q_quat_actual');
% xlabel('Tempo [s]');
% ylabel('Quaternioni');
% title('Quaternioni dell''End-Effector');
% legend('q_1', 'q_2', 'q_3', 'q_4');
% grid on;
% 
% %% Salvataggio Risultati
% 
% save('data\simulazione_traiettoria.mat', ...
%     'q_actual', 'qdot_actual', 'q2dot_des', ...
%     'ee_actual', 'ee_desired', 'e_position', 'e_orientation', ...
%     'q_eul_actual', 'q_quat_actual', 't_total');
