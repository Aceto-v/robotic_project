%Progetto Robotica Industriale - Prendibotv12
%Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Progettare e simulare il movimento di un braccio robotico 6DOF il cui
% compito assegnato: parti dalla posizione iniziale P0 -> vai alla
% posizione P1 e prendi oggetto -> sposta oggetto alla posizione P2 ->
% sposta oggetto alla posizione P3 -> torna alla posizione iniziale P0
%Script calcolo Cinemativa Inversa

%% Caricamento cartella Functions and Data
addpath("data")

load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Definizione parametri
% Inizializzazione dei parametri
a2 = 0.5; % [m]
d4 = 0.4; % [m]
d6 = 0.3; % [m]

% Calcola i punti specifici nel workspace
[pos_iniziale, p_alto, p_meta_altezza, p_terra] = workpoint(workspace);

% Salva i risultati dei punti
save('data\punti_workpoint.mat', 'pos_iniziale', 'p_alto', 'p_meta_altezza', 'p_terra');

% Matrice di rotazione desiderata
R_desiderata = eye(3);

%% Calcolo angoli giunture
% Calcolo delle configurazioni delle giunture per tutte le posizioni desiderate
q_iniziale = calcola_angoli(pos_iniziale, R_desiderata, a2, d4, d6);
q_alto = calcola_angoli(p_alto, R_desiderata, a2, d4, d6);
q_meta_altezza = calcola_angoli(p_meta_altezza, R_desiderata, a2, d4, d6);
q_terra = calcola_angoli(p_terra, R_desiderata, a2, d4, d6);

%Stampa matrici di rotazione e trasformazione
display_matrix(q_iniziale, q_alto, q_meta_altezza, q_terra, R_desiderata);

% Salva i risultati
save('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');

%% Chaos Version
% % Inizializzazione dei parametri
% a2 = 10; 
% d4 = 10; 
% d6 = 10; 
% 
% % Carica il workspace calcolato precedentemente
% load('data\prendibotv12_workspace.mat', 'workspace');
% 
% % Seleziona un certo numero di posizioni casuali dal workspace
% num_positions = 4; % Numero di posizioni desiderate
% selected_indices = randperm(size(workspace, 1), num_positions);
% selected_positions = workspace(selected_indices, :);
% 
% % Matrice di rotazione desiderata (identità in questo esempio)
% R_desiderata = eye(3);
% 
% % Calcolo delle configurazioni delle giunture per tutte le posizioni desiderate
% q_positions = zeros(num_positions, 6);
% for i = 1:num_positions
%     q_positions(i, :) = calcola_angoli(selected_positions(i, :), R_desiderata, a2, d4, d6);
% end
% 
% %Stampa matrici di rotazione e trasformazione
% display_matrix(q_positions, R_desiderata);
% 
% % Salva i risultati
% save('data\risultati_invKin_Prendibotv12.mat', 'q_positions', 'selected_positions');
% 
% 
% % Stampa le configurazioni trovate
% disp('Configurazioni delle giunture:');
% disp(q_positions);
