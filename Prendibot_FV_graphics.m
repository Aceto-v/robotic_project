% Prendibot_FV_graphics
% Script per analizzare i dati della simulazione

% Carica i dati della simulazione
load('prendibot_trajectory_analysis.mat', 'traj_endeffector', 'errors_position', 'errors_orientation', 'joint_velocities', 'joint_accelerations', 'singularity_values', 'q_trajectory');

% Visualizzazione degli errori di posizione
figure;
plot(errors_position, 'b-');
xlabel('Passi');
ylabel('Errore di Posizione');
title('Errore di Posizione durante la Traiettoria');
grid on;

% Visualizzazione degli errori di orientamento
figure;
plot(errors_orientation, 'r-');
xlabel('Passi');
ylabel('Errore di Orientamento');
title('Errore di Orientamento durante la Traiettoria');
grid on;

% Visualizzazione delle velocità dei giunti
figure;
plot(joint_velocities, 'g-');
xlabel('Passi');
ylabel('Velocità dei Giunti');
title('Velocità dei Giunti durante la Traiettoria');
grid on;

% Visualizzazione delle accelerazioni dei giunti
figure;
plot(joint_accelerations, 'm-');
xlabel('Passi');
ylabel('Accelerazione dei Giunti');
title('Accelerazione dei Giunti durante la Traiettoria');
grid on;

% Visualizzazione dei valori singolari
figure;
plot(singularity_values);
xlabel('Passi');
ylabel('Valori Singolari del Jacobiano');
title('Valori Singolari del Jacobiano durante la Traiettoria');
grid on;

% Analisi della singolarità
singular_values = zeros(size(q_trajectory, 1), 1);
for i = 1:size(q_trajectory, 1)
    singular_values(i) = min(svd(Rob.jacob0(q_trajectory(i, :))));
end

figure;
plot(singular_values, 'k-');
xlabel('Passi');
ylabel('Valore Singolare Minimo');
title('Analisi della Singolarità durante la Traiettoria');
grid on;

disp('Analisi grafica completata.');

