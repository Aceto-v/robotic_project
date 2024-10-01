%% Display Matrix Script
function display_matrix(q_iniziale, q_alto, q_meta_altezza, q_terra, R_desiderata)
    % Stampa della matrice di rotazione desiderata
    disp('Matrice di rotazione desiderata R_desiderata:');
    disp(R_desiderata);
    
    % Configurazioni e posizioni
    posizioni = {'Iniziale', 'Alto', 'Meta Altezza', 'Terra'};
    q_totali = {q_iniziale, q_alto, q_meta_altezza, q_terra};

    for i = 1:length(posizioni)
        fprintf('\n--- Configurazione %s ---\n', posizioni{i});
        q = q_totali{i};

        % Calcolo delle matrici di rotazione
        theta_1 = q(1);
        theta_2 = q(2);
        theta_3 = q(3);

        R0_1 = trotz(theta_1) * trotx(pi/2);
        disp('Matrice di rotazione R0_1:');
        disp(R0_1);

        R1_2 = trotz(theta_2);
        disp('Matrice di rotazione R1_2:');
        disp(R1_2);

        R2_3 = trotz(theta_3) * trotx(pi/2);
        disp('Matrice di rotazione R2_3:');
        disp(R2_3);

        R0_3 = R0_1 * R1_2 * R2_3;
        disp('Matrice di rotazione R0_3:');
        disp(R0_3);

        R0_3_rot = R0_3(1:3, 1:3);
        disp('Matrice di rotazione estratta R0_3_rot:');
        disp(R0_3_rot);

        R3_6 = R0_3_rot' * R_desiderata;
        disp('Matrice di rotazione R3_6:');
        disp(R3_6);
    end
end
