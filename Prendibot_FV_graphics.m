%% Prendibot_FV_graphics
% Script per analizzare i dati della simulazione

%% Caricamento cartella Functions and Data
addpath("data")

% Carica i dati della simulazione
load('data\prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory', 't_total', 'dt');

%% Tempo
%Creazione asse temporale
time = (0:dt:t_total);
time = time(1:length(errors_position));

%% Grafici errore di posizione e errore di orientamento
% Visualizzazione degli errori di posizione e orientamento
figure
subplot(2, 1, 1)
    plot(time, errors_position, 'r', 'LineWidth', 2.0)
    grid on
    xlabel("Tempo [s]")
    ylabel("Errore di Posizione [m]")
    title("Errore di Posizione durante la Traiettoria")

subplot(2, 1, 2)
    plot(time, errors_orientation, 'b', 'LineWidth', 2.0)
    grid on
    xlabel("Tempo [s]")
    ylabel("Errore di Orientamento [m]")
    title("Errore di Orientamento durante la Traiettoria")

%% Grafici velocità dei giunti e accellerazione dei giunti durante la triettoria
% Viualizzazione Velocità e accellerazione dei giunti
figure
subplot(2, 1, 1)
    plot(time, joint_velocities, 'g', 'LineWidth', 2.0)
    grid on
    xlabel("Tempo [s]")
    ylabel("Velocità dei Giunti [rad/s]")
    title("Velocità dei Giunti durante la Traiettoria")

subplot(2, 1, 2)
    plot(time, joint_accelerations, 'm', 'LineWidth', 2.0)
    grid on
    xlabel("Tempo [s]")
    ylabel("Accelerazione dei Giunti [rad/s^2]")
    title("Accelerazione dei Giunti durante la Traiettoria")

%% Grfici Singolarità

% Analisi della singolarità
singular_values = zeros(size(q_trajectory, 1), 1);
for i = 1:size(q_trajectory, 1)
    singular_values(i) = min(svd(Rob.jacob0(q_trajectory(i, :))));
end

% Visualizzazione dei valori singolari
figure
subplot(2, 1, 1)
    plot(time, singularity_values, 'LineWidth', 2.0);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Valori Singolari del Jacobiano');
    title('Valori Singolari del Jacobiano durante la Traiettoria');

subplot(2, 1, 2)
    plot(time, singular_values, 'k', 'LineWidth', 2.0);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Valore Singolare Minimo');
    title('Analisi della Singolarità durante la Traiettoria');
    grid on;

disp('Analisi grafica completata.');


