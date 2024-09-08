%% Prendibot_FV_graphics
% Script per analizzare i dati della simulazione

%% Caricamento cartella Functions and Data
addpath("data")

% Carica i dati della simulazione
load('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory');

%% Grafici errore di posizione e errore di orientamento
% Visualizzazione degli errori di posizione e orientamento
figure
subplot(2, 1, 1)
    plot(errors_position, 'r', 'LineWidth', 2.0)
    grid on
    xlabel("Passi")
    ylabel("Errore di Posizione")
    title("Errore di Posizione durante la Traiettoria")

subplot(2, 1, 2)
    plot(errors_orientation, 'b', 'LineWidth', 2.0)
    grid on
    xlabel("Passi")
    ylabel("Errore di Orientamento")
    title("Errore di Orientamento durante la Traiettoria")
% Old version
% Visualizzazione degli errori di posizione
% figure;
% plot(errors_position, 'b-');
% xlabel('Passi');
% ylabel('Errore di Posizione');
% title('Errore di Posizione durante la Traiettoria');
% grid on;
% Visualizzazione degli errori di orientamento
% figure;
% plot(errors_orientation, 'r-');
% xlabel('Passi');
% ylabel('Errore di Orientamento');
% title('Errore di Orientamento durante la Traiettoria');
% grid on;

%% Grafici velocità dei giunti e accellerazione dei giunti durante la triettoria
% Viualizzazione Velocità e accellerazione dei giunti
figure
subplot(2, 1, 1)
    plot(joint_velocities, 'g', 'LineWidth', 2.0)
    grid on
    xlabel("Passi")
    ylabel("Velocità dei Giunti")
    title("Velocità dei Giunti durante la Traiettoria")

subplot(2, 1, 2)
    plot(joint_accelerations, 'm', 'LineWidth', 2.0)
    grid on
    xlabel("Passi")
    ylabel("Accelerazione dei Giunti")
    title("Accelerazione dei Giunti durante la Traiettoria")
% Old version
% % Visualizzazione delle velocità dei giunti
% figure;
% plot(joint_velocities, 'g-');
% xlabel('Passi');
% ylabel('Velocità dei Giunti');
% title('Velocità dei Giunti durante la Traiettoria');
% grid on;
% % Visualizzazione delle accelerazioni dei giunti
% figure;
% plot(joint_accelerations, 'm-');
% xlabel('Passi');
% ylabel('Accelerazione dei Giunti');
% title('Accelerazione dei Giunti durante la Traiettoria');
% grid on;

%% Grfici Singolarità

% Analisi della singolarità
singular_values = zeros(size(q_trajectory, 1), 1);
for i = 1:size(q_trajectory, 1)
    singular_values(i) = min(svd(Rob.jacob0(q_trajectory(i, :))));
end

% Visualizzazione dei valori singolari
figure
subplot(2, 1, 1)
    plot(singularity_values, 'LineWidth', 2.0);
    grid on;
    xlabel('Passi');
    ylabel('Valori Singolari del Jacobiano');
    title('Valori Singolari del Jacobiano durante la Traiettoria');

subplot(2, 1, 2)
    plot(singular_values, 'k', 'LineWidth', 2.0);
    grid on;
    xlabel('Passi');
    ylabel('Valore Singolare Minimo');
    title('Analisi della Singolarità durante la Traiettoria');
    grid on;

disp('Analisi grafica completata.');

