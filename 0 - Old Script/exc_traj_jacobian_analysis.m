%% Click version (Vicio)
function [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = exc_traj_jacobian_analysis(Rob, q_start, q_end, t_total, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v)
    
    % Generazione della traiettoria
    [q_des, qdot_des] = jtraj(q_start, q_end,t_total);

    % Imposta il guadagno per il controller CLIK
    clik_gain = 1e5;

    % Ciclo nel tempo di simulazione
    for i = 1:t_total

        % Calcolo dello Jacobiano (q_des)
        J_des = Rob.jacob0(q_des(i, :));

        % Velocità desiderata dell'end-effector
        ee_des_vel = J_des*qdot_des(i, :)';

        % Inverse Kinematics (Open-Loop)
        J_inv = pinv(J_des + epsilon * eye(size(J_des)));

        % Errore tra la velocità desiderata e quella effettiva
        error = ee_des_vel - J_des*qdot_des(i, :)';  
        
        % CLIK control law
        q_dot = clik(Rob, q_des(i, :), error, ee_des_vel, clik_gain);

        % Aggiornamento posizione giunti
        q_des(i, :) = q_des(i, :) + (q_dot' * dt);

        % Applicazione dei limiti di giunto
        q_des(i, :) = max(Rob.qlim(:,1)', min(Rob.qlim(:,2)', q_des(i, :)));

        % Forward kinematics
        T = Rob.fkine(q_des(i, :));
        traj_endeffector = [traj_endeffector; T.t'];
        
        % % Calcolo degli errori di posizione e orientamento
        % T_current = Rob.fkine(q_des(i, :));
        % pos_error = norm(T_current.t' - traj_endeffector(i, 1:3));
        % orient_error = norm(tr2rpy(T_current.R) - tr2rpy(Rob.fkine(q_des(i, :)).R));
        % errors_position = [errors_position; pos_error];
        % errors_orientation = [errors_orientation; orient_error];

        % Errore di posizionamento
        T_current = Rob.fkine(q_des(i, :));
        pos_error = norm(T_current.t' - q_des(i, 1:3));
        errors_position = [errors_position; pos_error];

        % Errore di orientamento
        T_des = Rob.fkine(q_end);
        orient_error = norm(tr2rpy(T_current.R) - tr2rpy(T_des.R));
        errors_orientation = [errors_orientation; orient_error];


        % %Old version
        % % Errore di posizionamento
        % T_current = Rob.fkine(q_des(i, :));
        % pos_error = norm(T_current.t' - q_des(i, 1:3));
        % errors_position = [errors_position; pos_error];
        % 
        % % Errore di orientamento
        % orient_error = norm(tr2rpy(T_current.R) - tr2rpy(Rob.fkine(q_des(i, :)).R));
        % errors_orientation = [errors_orientation; orient_error];

        % Velocità dei giunti
        joint_vel = norm(q_dot);
        joint_velocities = [joint_velocities; joint_vel];

        % Accellerazione dei giunti 
        if i > 1
            joint_acc = (joint_vel - joint_velocities(end-1)) / dt;
            joint_accelerations = [joint_accelerations; joint_acc];
        else
            joint_accelerations = [joint_accelerations; 0];
        end

        % Valori singolari dello Jacobiano
        singular_val = svd(J_des);
        singularity_values = [singularity_values; singular_val'];

        % Traiettoria dei giunti
        q_trajectory = [q_trajectory; q_des(i, :)];

        % Simulazione del movimento del robot
        if mod(i, 5) == 0
            Rob.plot(q_des(i, :));
            plot3(traj_endeffector(:,1), traj_endeffector(:,2), traj_endeffector(:,3), 'r-', 'LineWidth', 2);
            frame = getframe(gcf);
            writeVideo(v, frame); % Scrive il frame nel video
            pause(0.1); % Aggiunta di una pausa per la visualizzazione
        end
    end

    % Visualizza i segnalini per le posizioni chiave
    plot3(T.t(1), T.t(2), T.t(3), 'go', 'MarkerSize', 5, 'MarkerFaceColor', '#7E2F8E');

    % Verifica se la posizione finale raggiunta è quella desiderata
    disp('Posizione finale raggiunta:');
    disp(q_des(end, :));
    disp('Posizione finale desiderata:');
    disp(q_end);
end

%% OLD TRY

% %% Previus version
% function [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = exc_traj_jacobian_analysis(Rob, q_start, q_end, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v)
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
%     % Verifica se la posizione finale raggiunta è quella desiderata
%     disp('Posizione finale raggiunta:');
%     disp(q);
%     disp('Posizione finale desiderata:');
%     disp(q_end);
% end

% %% NEW Function (Dani's version)
% function [traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory] = esegui_traiettoria_jacobian_analisi(Rob, q_start, q_end, num_steps, dt, epsilon, traj_endeffector, errors_position, errors_orientation, joint_velocities, joint_accelerations, singularity_values, q_trajectory, v)
%     %% Trajectory Generation
%     [q_des, qdot_des, q2dot_des] = jtraj(q_start, q_end, num_steps);
% 
%     % Gain for the Inverse1 Kinematics controller
%     clik_gain = e2*eye(size(q_des, 2));
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

% %% OLD (Vicio's Fuction)
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