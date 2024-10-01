%Progetto Robotica Industriale - Prendibotv12
%Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Progettare e simulare il movimento di un braccio robotico 6DOF il cui
% compito assegnato: parti dalla posizione iniziale P0 -> vai alla
% posizione P1 e prendi oggetto -> sposta oggetto alla posizione P2 ->
% sposta oggetto alla posizione P3 -> torna alla posizione iniziale P0
%Script calcolo workspace

%% Creazione braccio robotico 6DOF
% Inizializza il braccio robotico con i parametri forniti
a2 = 0.5; % [m]
d4 = 0.4; % [m]
d6 = 0.3; % [m]

L1 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi pi]);
L2 = Link('d', 0, 'a', a2, 'alpha', 0, 'qlim', [-pi/2 pi/2]);
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/2 pi/2]);
L4 = Link('d', d4, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi/2 pi/2]);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/2 pi/2]);
L6 = Link('d', d6, 'a', 0, 'alpha', 0, 'qlim', [-pi pi]);

Rob = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Rob');

%% Creazione dello spazio di lavoro del braccio
% Parametri per il calcolo del workspace
num_points = 1e+3; % Numero di punti da generare

% Genera valori casuali per i giunti entro i loro limiti
q1 = (L1.qlim(2) - L1.qlim(1)) * rand(num_points, 1) + L1.qlim(1);
q2 = (L2.qlim(2) - L2.qlim(1)) * rand(num_points, 1) + L2.qlim(1);
q3 = (L3.qlim(2) - L3.qlim(1)) * rand(num_points, 1) + L3.qlim(1);
q4 = (L4.qlim(2) - L4.qlim(1)) * rand(num_points, 1) + L4.qlim(1);
q5 = (L5.qlim(2) - L5.qlim(1)) * rand(num_points, 1) + L5.qlim(1);
q6 = (L6.qlim(2) - L6.qlim(1)) * rand(num_points, 1) + L6.qlim(1);

% Inizializza l'array per memorizzare le posizioni dell'endeffector
workspace = zeros(num_points, 3);

% Calcola la posizione dell'endeffector per ogni configurazione dei giunti
for i = 1:num_points
    q = [q1(i), q2(i), q3(i), q4(i), q5(i), q6(i)];
    T = Rob.fkine(q);
    workspace(i, :) = T.t';
end

% Salva il robot e il workspace
save('data\prendibot_workspace.mat', 'Rob', 'workspace');