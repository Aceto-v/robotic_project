%%% Prendibot Simulation %%%
clear all
close all
clc

%% Load Functions and Data folders
% In questa sezione, carico i dati calcolati dagli altri programmi e funzioni utili. %

addpath("functions")
addpath("data")
addpath("video\")

% Inverse Kinematics Data
load('data\risultati_invKin_Prendibotv12.mat', 'q_iniziale', 'q_alto', 'q_meta_altezza', 'q_terra');
% Robot object and Workspace Analysis
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Define Simulation Parameters
% In questa sezione, sto definendo tutti i parametri utili per la
% simulazione.
% 1) Time Settings: qui definisco quanto dura la mia simulazione (tf),
% a quanto sto campionando (fs) e dunque tutti gli istanti di tempo (t).

% 2) Regularization: durante l'inversione del jacobiano, può succedere che
% il jacobiano sia MOLTO vicino all'essere non invertibile e dunque la sua
% inversione numerica può portare un risultato molto alto.
% Infatti, inv(J) = (1/det(J))*adj(J), dove il det(J) è molto vicino allo
% zero. Di conseguenza, questa inversione avrà numeri molto alti,
% rischiando di mandare a tarallucci e vino il controllore ad inversione
% cinematica. 
% Un modo per metterci una pezza è quello di sommare un epsilon per rendere 
% meno vicino alla singolarità la matrice.
% Questo è un metodo semplice che però ovviamente "sporca" il jacobiano
% dalla sua correttezza. Quindi più si alza il valore di epsilon, più il
% jacobiano non sarà esattamente quello corretto e l'inversione cinematica
% non sarà più esatta. Più epsilon è piccolo, più sarà precisa l'inversione
% ma ovviamente si rischia di avere il problema di inversione numerica sopracitato.

% 3) Initial Condition: Per risolvere qualsiasi integrazione numerica, è
% necessario definire le condizioni iniziali del nostro robot. Per
% semplicità, qui ho messo che tutti i giunti sono posizionati come quelli
% della nostra traiettoria desiderata. Alternativamente, per complicare il
% tutto, si potrebbe anche scegliere che i valori di giunto siano casuali.

% Configurations
config_sequence = {q_iniziale, q_alto, q_meta_altezza, q_terra, q_iniziale};
n_configs = length(config_sequence);

% Time Settings
tf = 1e1*(n_configs - 1);    % [s]
fs = 1e2;    % [Hz]
t = (0:(1/fs):tf)';

% Regularization
epsilon = 1e-3;     % avoid numerical singularities

% Init Condition
q0 = q_iniziale';
% q0 = randn(length(q_iniziale), 1);
qdot0 = zeros(length(q0), 1);

%% Generate Trajectory
% Dopo aver generato le traiettorie nello spazio di giunti con la funzione
% jtraj(), le visualizziamo.
% P.s.: lo spazio dei giunti o spazio di configurazione è l'insieme di
% tutte le q che il robot può assumere.

% Init variables
q_des = [];
qdot_des = [];
q2dot_des = [];

% Stuck the desired confs
for i = 1:(n_configs - 1)
    [q_act_des, qdot_act_des, q2dot_act_des] = jtraj(config_sequence{i}, config_sequence{i + 1}, t(1:round(length(t)/(n_configs - 1))));
    
    % Update
    q_des = [q_des; q_act_des];
    qdot_des = [qdot_des; qdot_act_des];
    q2dot_des = [q2dot_des; q2dot_act_des];
end
clear q_act_des qdot_act_des q2dot_act_des

q_des = [q_des; q_des(end, :)];
qdot_des = [qdot_des; qdot_des(end, :)];
q2dot_des = [q2dot_des; q2dot_des(end, :)];

% Traspongo perchè mi da fastidio a righe, mannaggia a chi a fatto jtraj
q_des = q_des';
qdot_des = qdot_des';
q2dot_des = q2dot_des';

%% Convert generated trajectory in Task Space
% Dato che il controllore ad inversione cinematica si basa su una
% traiettoria desiderata dell'end-effector, bisogna passare da un
% traiettoria desiderata in q in una di posizione ed orientamento del
% manipolatore.
% Come parameterizzazione di orientamento, ho scelto la classica RPY
% (roll-pitch-yaw). Tuttavia, se uno vuole essere generale, dovrebbe
% utilizzare i quaternioni, che non hanno il problema del gimbal lock.

% Per avere informazioni sulla velocità lineare ed angolare desiderata,
% bisogna utilizzare il jacobiano. Da cui:
% [v; omega] = J(q)*q_dot
% Nota che qui non dobbiamo fare alcuna inversione, quindi non occorre
% "sporcare" il jacobiano con epsilon.

% Successivamente, dato che la nostra traiettoria è riferita agli angoli di
% eulero, dobbiamo utilizzare la relazione che lega la velocità angolare
% alla derivata temporale degli angoli. Questa trasformazione è esattamente
% quella che converte un jacobiano geometrico (ovvero quello relativo a
% omega) ad uno analitico (ovvero relativo alla derivata temporale degli
% angoli RPY).

% Init Position and Orientation
p_des = zeros(3, length(t));    % Position [m]
eul_des = zeros(3, length(t));    % Euler RPY [rad]
xi_des = zeros(6, length(t));           % Generalized velocity vector

for i = 1:length(t)
    % Compute Forward and Differential Kinematics
    T = Rob.fkine(q_des(:, i));
    J = Rob.jacob0(q_des(:, i));
    % Ja = ;

    % Extract Pose (Position + Orientation)
    p_des(:, i) = T.t;
    eul_des(:, i) = rotm2eul(T.R);

    % Extract the Velocity Twist
    xi_des(:, i) = J*qdot_des(:, i);
end

%% Simulation
% Il nostro obiettivo qui è simulare la cinematica differenziale del robot
% con controllore CLIK (Closed-Loop Inverse Kinematics).
% Il CLIK assume di poter controllare direttamente le
% velocità dei motori dei giunti (q_dot = u). Questo spesso è vero anche in
% pratica, dove i controllori a basso livello dei motori elettrici
% permettono di comandarne direttamente la velocità.

% La legge di controllo del CLIK si può scrivere come:
% u = inv(J)*(desired_pose_dot + K*error)
% - Il valore K è il gain del controllore, il quale ti permette di regolarne
% le performance.
% - Il termine desired_pose_dot è un'azione in feedforward. L'informazione
% della derivata della traiettoria ti da infatti un'informazione di come
% la traiettoria si evolve nei tempi successivi, dandoti il tempo al passo
% precedente di reagire.

% Nota sull'integrazione:
% Un modo semplice per risolvere un'integrazione numerica è il metodo di
% "Eulero in avanti", o in inglese "Forward Euler".
% Il metodo si basa sulla definizione della derivata.
% IN questo caso,
% q_dot = lim(dt -> 0) (q(t + dt) - q(t))/(dt).
% Se consideriamo nel discreto, la derivata può essere approssimata come:
% q_dot ~ (q(k) - q(k - 1))/T_s,
% dove T_s è il periodo di campionamento. Invertendo la relazione:
% q(k) = q(k - 1) + Ts*(q_dot).

% Timeseries of the joint variables of the real robot
q = q0;
qdot = qdot0;
p_robot = Rob.fkine(q).t;
% K = 1e1;
K = 1e1;

% Aggiunte vicio
% Qui vado a creare il file video, iniziando con la definizione
% dell'oggetto video, poi procederò all'interno del ciclo di simulazione
% (dopo ogni aggiornamento della posizione del robot) con l'aggiunta del
% comando per catturare il frame della figura, infine chiudo il video
% Definizione dell'oggetto VideoWriter
video_filename = 'video\Prendibot_simulation.mp4'; % Definizione del nome del file e il formato
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fs; % Impostazione del framerate al valore di campionamento fs
open(v);
% Inizializza la figura per la visualizzazione
figure;
Rob.plot(q_iniziale); % Visualizzazione della configurazione iniziale
hold on;

% Start Cycling
for i = 1:(length(t) - 1)
    % Calcolo il jacobiano alla configurazione attuale
    J = Rob.jacob0(q(:, i));
    % Regolarizzazione
    J = J + epsilon*eye(size(J));
    
    % Select Position Jacobian
    Jp = J(1:3, :);
    % Position Error
    ep = p_des(:, i) - p_robot(:, i);
    qdot(:, i) = Jp \ (xi_des(1:3, i) + K*(ep));


    % Integration
    % q(k) = q(k - 1) + Ts*q_dot(k - 1)
    q(:, i + 1) = q(:, i) + (1/fs)*qdot(:, i);

    %Update Robot Position
    p_robot(:, i + 1) = Rob.fkine(q(:, i + 1)).t;
    eul_robot(:, i + 1) = rotm2eul(Rob.fkine(q(:, i + 1)).R);

    %(Aggiunta Vicio) Plot aggiornato per ogni passo
    if mod(i, 5) == 0
        figure(1); % Use a specific figure for the robot plot
        Rob.plot(q(:, i)');
        hold on;
        plot3(p_robot(1, 1:i), p_robot(2, 1:i), p_robot(3, 1:i), 'r-', 'LineWidth', 2);
        hold off;
        
        % Capture the current frame for the video
        frame = getframe(gcf);  
        writeVideo(v, frame);
        pause(0.05);
    end
    
    figure(100); clf;
    plot3(p_des(1, :),  p_des(2, :), p_des(3, :), 'LineWidth', 2.0, 'LineStyle', '--');
    hold on;
    plot3(p_robot(1, 1:i),  p_robot(2, 1:i), p_robot(3, 1:i), 'LineWidth', 2.0);
    grid on;
    xlabel("x [m]");
    ylabel("y [m]");
    zlabel("z [m]");
    legend("Desired Trajectory", "EE Position");
    title("Desired and Real Trajectory");
    hold off;
end

%(Aggiunta Vicio) chiusura video
close(v);

%% Plot Result
figure
plot(t, p_des - p_robot, 'LineWidth', 2.0)
grid on
xlabel("t [s]")
ylabel("EE position error [m]")
legend("x", "y", "z")
title("EE position error")

figure
plot(t, wrap2pi(eul_des - eul_robot), 'LineWidth', 2.0)
grid on
xlabel("t [s]")
ylabel("EE orientation error [rad]")
legend("roll", "pitch", "yaw")
title("EE orientation error")

figure
plot3(p_des(1, :),  p_des(2, :), p_des(3, :), 'LineWidth', 2.0, 'LineStyle', '--')
hold on
plot3(p_robot(1, :),  p_robot(2, :), p_robot(3, :), 'LineWidth', 2.0)
hold off
grid on
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
legend("Desired Trajectory", "EE Position")
title("Desired and Real Trajectory")

% Nota sul wrap2pi:
% Quando si esegue una differenza di angoli, potrebbe succedere che fai
% pi  - (- pi) = 2pi. L'errore tuttavia non è 0 anche se pi e -pi sono lo
% stesso numero, se si definiscono gli angoli nel range [-pi, pi].
% Se si immagina anche un controllore proporzionale alla differenza di
% errore, produrrebbe una reazione ad un errore di 2pi (che sarebbe anche
% molto alto) in una situazione in cui invece in realtà l'errore sarebbe
% nullo.

% Per ritraslare tutti gli angoli dentro il range di definizione, si usa
% solitamente fare:
% angle_error = atan2(sin(angle2 - angle1), cos(angle2 - angle1));

%% Functions
function wrapped_angle = wrap2pi(angle)
    wrapped_angle = atan2(sin(angle), cos(angle));
end
