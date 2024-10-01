%Progetto Robotica Industriale - Prendibotv12
%Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Progettare e simulare il movimento di un braccio robotico 6DOF il cui
% compito assegnato: parti dalla posizione iniziale P0 -> vai alla
% posizione P1 e prendi oggetto -> sposta oggetto alla posizione P2 ->
% sposta oggetto alla posizione P3 -> torna alla posizione iniziale P0
%Script calcolo Cinemativa Inversa

%% Caricamento cartella Functions and Data
addpath("data")

load('data\prendibot_workspace.mat', 'Rob', 'workspace');

%% Definizione parametri
% Inizializzazione dei parametri
a2 = 0.5; % [m]
d4 = 0.4; % [m]
d6 = 0.3; % [m]

% Calcola i punti specifici nel workspace
[pos_iniziale, p_alto, p_meta_altezza, p_terra] = workpoint(workspace);

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

%% Salvataggio dati

% Salva i risultati dei punti
save('data\punti_workpoint.mat', 'pos_iniziale', 'p_alto', 'p_meta_altezza', 'p_terra');

% Salva i risultati degli angoli
save('data\risultati_invKin_Prendibot.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');