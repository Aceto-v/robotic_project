%Progetto Robotica Industriale - Prendibotv12
%Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Progettare e simulare il movimento di un braccio robotico 6DOF il cui
% compito assegnato: parti dalla posizione iniziale P0 -> vai alla
% posizione P1 e prendi oggetto -> sposta oggetto alla posizione P2 ->
% sposta oggetto alla posizione P3 -> torna alla posizione iniziale P0
%Script calcolo Cinemativa Inversa

% Inizializzazione dei parametri
a2 = 10; 
d4 = 10; 
d6 = 10; 

% Posizioni desiderate
pos_iniziale = [15, 0, 10];
p_alto = [-17, 0, 40]; % Punto molto in alto
p_meta_altezza = [15, 0, 20]; % Punto opposto e a metà altezza
p_terra = [6, -14, 5]; % Punto per terra

% Matrice di rotazione desiderata (identità in questo esempio)
R_desiderata = eye(3);

% Calcolo delle configurazioni delle giunture per tutte le posizioni desiderate
q_iniziale = calcola_angoli(pos_iniziale, R_desiderata, a2, d4, d6);
q_alto = calcola_angoli(p_alto, R_desiderata, a2, d4, d6);
q_meta_altezza = calcola_angoli(p_meta_altezza, R_desiderata, a2, d4, d6);
q_terra = calcola_angoli(p_terra, R_desiderata, a2, d4, d6);

%stampa le matrici di trasformazione e rotazione
display_matrix(q_iniziale, q_alto, q_meta_altezza, q_terra, R_desiderata);

% Salva i risultati
save('risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');

% Funzione per calcolare gli angoli delle giunture dato una posizione e orientazione desiderate
function q = calcola_angoli(pos, orientazione, a2, d4, d6)

    % Calcolo della posizione del polso
    p_w = pos - [0, 0, d6];
    
    % Calcolo di theta1
    theta_1 = atan2(p_w(2), p_w(1));
    
    % Calcolo di r e z_w
    r = sqrt(p_w(1)^2 + p_w(2)^2);
    z_w = p_w(3);
    
    % Calcolo di c3
    c3 = (r^2 + z_w^2 - a2^2 - d4^2) / (2 * a2 * d4);
    c3 = max(min(c3, 1), -1); % Verifica dei limiti di c3 per evitare errori di dominio
    s3 = sqrt(1 - c3^2);
    
    % Calcolo di theta3
    theta_3 = atan2(s3, c3);
    
    % Calcolo di theta2
    theta_2 = atan2(z_w, r) - atan2(d4 * s3, a2 + d4 * c3);
    
    % Matrici di rotazione dei primi tre giunti
    R0_1 = trotz(theta_1) * trotx(pi/2);    %R0_1 = [cos(theta_1), -sin(theta_1), 0; sin(theta_1), cos(theta_1), 0; 0, 0, 1] * [1, 0, 0; 0, 0, -1;  0, 1, 0];
    R1_2 = trotz(theta_2);                  %R1_2 = [cos(theta_2), -sin(theta_2), 0; sin(theta_2), cos(theta_2), 0; 0, 0, 1];
    R2_3 = trotz(theta_3) * trotx(pi/2);    %R2_3 = [cos(theta_3), -sin(theta_3), 0; sin(theta_3), cos(theta_3), 0; 0, 0, 1] * [1, 0, 0; 0, 0, -1; 0, 1, 0];
    
    % Matrice di rotazione risultante dalla cinematica diretta dei primi tre giunti
    R0_3 = R0_1 * R1_2 * R2_3;
    
    % Estrarre la parte di rotazione 3x3 da R0_3
    R0_3_rot = R0_3(1:3, 1:3);
    
    % Calcolo della matrice di rotazione necessaria per gli ultimi tre giunti
    R3_6 = R0_3_rot' * orientazione;
    
    % Calcolo degli angoli del polso (theta4, theta5, theta6)
    theta_4 = atan2(R3_6(2,3), R3_6(1,3));
    theta_5 = atan2(sqrt(R3_6(1,3)^2 + R3_6(2,3)^2), R3_6(3,3));
    theta_6 = atan2(R3_6(3,2), -R3_6(3,1));
    
    % Angoli delle giunture
    q = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6];
    
end