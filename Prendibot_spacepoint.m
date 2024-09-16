% Script definizione punti specifici

%% Carica il workspace
load('data\prendibotv12_workspace.mat', 'Rob', 'workspace');

%% Trova i limiti del workspace
x_min = min(workspace(:,1));
x_max = max(workspace(:,1));
y_min = min(workspace(:,2));
y_max = max(workspace(:,2));
z_min = min(workspace(:,3));
z_max = max(workspace(:,3));

%% Trova i punti più alti su uno dei due lati
% Trova i punti vicino al bordo laterale destro e sinistro
side_threshold = 0.1; % Soglia per considerare un punto come "vicino al lato"
right_side = workspace(:,1) > (x_max - side_threshold);
left_side = workspace(:,1) < (x_min + side_threshold);

% Trova i punti più alti a destra e a sinistra
[~, idx_right] = max(workspace(right_side, 3));
[~, idx_left] = max(workspace(left_side, 3));

p_alto_right = workspace(right_side, :);
p_alto_left = workspace(left_side, :);

p_alto_right = p_alto_right(idx_right, :);
p_alto_left = p_alto_left(idx_left, :);

% Scegli il punto più alto su uno dei lati
if norm(p_alto_right - [mean(workspace(:,1)), mean(workspace(:,2)), z_max]) < norm(p_alto_left - [mean(workspace(:,1)), mean(workspace(:,2)), z_max])
    p_alto = p_alto_right;
else
    p_alto = p_alto_left;
end

%% Trova il punto opposto e a metà altezza rispetto a p_alto (p_meta_altezza)
% Definisci il lato opposto rispetto a p_alto
if p_alto(1) > (x_max - side_threshold)
    opposite_side = workspace(:,1) < (x_min + side_threshold);
else
    opposite_side = workspace(:,1) > (x_max - side_threshold);
end

% Trova i punti a metà altezza
half_height = (z_max + z_min) / 2;
opposite_points = workspace(opposite_side, :);
[~, idx_opposite] = min(abs(opposite_points(:,3) - half_height)); % Trova il punto più vicino a metà altezza

p_meta_altezza = opposite_points(idx_opposite, :);

%% Definisci p_terra (leggermente sopra il suolo e spostato verso il bordo)
% Trova il punto più vicino al bordo dell'area di lavoro
border_threshold = 0.1; % Distanza dal bordo
border_points = workspace;
border_points(border_points(:,1) < (x_min + border_threshold) | border_points(:,1) > (x_max - border_threshold) | ...
              border_points(:,2) < (y_min + border_threshold) | border_points(:,2) > (y_max - border_threshold), :) = [];

% Se ci sono punti validi, scegli uno di questi punti per p_terra
if ~isempty(border_points)
    p_terra = border_points(1, :); % Usa il primo punto valido
else
    % Altrimenti, usa un punto vicino al centro come fallback
    p_terra = [mean(workspace(:,1)), mean(workspace(:,2)), z_min + 0.1]; % Aggiungi 0.1m sopra il suolo
end

%% Definizione della posizione iniziale
offset_altezza = 0.7; % Offset verticale per posizionare il robot leggermente sopra il suolo
offset_sicurezza = 0.3; % Offset orizzontale per evitare collisioni

% Posizione iniziale vicino a p_terra, ma leggermente sopra e a distanza di sicurezza
pos_iniziale = [p_terra(1) + offset_sicurezza, p_terra(2) + offset_sicurezza, p_terra(3) + offset_altezza];

% Supponiamo che l'orientamento sia neutro (zero rotazioni)
orientamento_iniziale = [0, 0, 0]; % Sostituisci con gli angoli di orientamento reali se necessari

%% Grafico dello spazio di lavoro e dei punti specifici
figure;
plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.');
hold on;
plot3(p_alto(1), p_alto(2), p_alto(3), 'ro', 'MarkerSize', 10, 'DisplayName', 'p\_alto');
plot3(p_meta_altezza(1), p_meta_altezza(2), p_meta_altezza(3), 'go', 'MarkerSize', 10, 'DisplayName', 'p\_meta\_altezza');
plot3(p_terra(1), p_terra(2), p_terra(3), 'mo', 'MarkerSize', 10, 'DisplayName', 'p\_terra');
plot3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3), 'ko', 'MarkerSize', 10, 'DisplayName', 'pos\_iniziale');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Workspace del Braccio Robotico con Punti Specifici');
grid on;
legend('Punti del WS', 'p\_alto', 'p\_meta\_altezza', 'p\_terra', 'pos\_iniziale', 'Location', 'best');
axis equal;
hold off;

%% Salva i punti
save('data\prendibotv12_workspace_punti.mat', 'pos_iniziale', 'p_alto', 'p_meta_altezza', 'p_terra');

