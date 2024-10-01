%% Workpoint Script
function [pos_iniziale, p_alto, p_meta_altezza, p_terra] = workpoint(workspace)
    % Funzione unica per calcolare e restituire pos_iniziale, p_alto, p_meta_altezza, p_terra
    
    % Trova il punto più alto sul lato del workspace
    p_alto = trova_punto_alto(workspace);
    
    % Trova il punto a metà altezza sul lato opposto di p_alto
    p_meta_altezza = trova_punto_meta_altezza(workspace, p_alto);
    
    % Trova un punto a livello del terreno sul lato opposto di p_alto e p_meta_altezza
    p_terra = trova_punto_terra(workspace, p_alto, p_meta_altezza);
    
    % Seleziona una posizione iniziale casuale ma che sia distante da p_alto, p_meta_altezza, e p_terra
    pos_iniziale = trova_pos_iniziale(workspace, p_alto, p_meta_altezza, p_terra);
end

%% Posizione Iniziale
function pos_iniziale = trova_pos_iniziale(workspace, p_alto, p_meta_altezza, p_terra)
    % Trova una posizione iniziale distante da p_alto, p_meta_altezza e p_terra
    tolleranza_altezza = 0.05; % Tolleranza per altezza non essere troppo vicini al livello del terreno
    
    % Filtra i punti che non siano troppo vicini alla base
    punti_validi = workspace(workspace(:, 3) > tolleranza_altezza, :);
    
    % Calcola le distanze laterali (X,Y) da p_alto, p_meta_altezza, e p_terra
    distanze_alto = sqrt((punti_validi(:, 1) - p_alto(1)).^2 + (punti_validi(:, 2) - p_alto(2)).^2);
    distanze_meta = sqrt((punti_validi(:, 1) - p_meta_altezza(1)).^2 + (punti_validi(:, 2) - p_meta_altezza(2)).^2);
    distanze_terra = sqrt((punti_validi(:, 1) - p_terra(1)).^2 + (punti_validi(:, 2) - p_terra(2)).^2);
    
    % Somma delle distanze da tutti i punti per massimizzare la distanza
    [~, idx_pos_iniziale] = max(distanze_alto + distanze_meta + distanze_terra);
    
    % Seleziona il punto iniziale che sia il più lontano possibile da tutti gli altri
    pos_iniziale = punti_validi(idx_pos_iniziale, :);
end

%% Punto Alto
function p_alto = trova_punto_alto(workspace)
    % Trova il punto più alto lateralmente nel workspace
    altezza_massima = max(workspace(:, 3)); % Trova altezza massima
    idx = find(workspace(:, 3) == altezza_massima); % Indici dei punti con altezza massima
    p_candidati = workspace(idx, :); % Punti candidati
    
    % Filtra i candidati per posizione laterale (non vicino all'asse centrale)
    distanza_centro = sqrt(p_candidati(:, 1).^2 + p_candidati(:, 2).^2);
    [~, idx_p_alto] = max(distanza_centro);
    p_alto = p_candidati(idx_p_alto, :);
end

%% Punto Metà Altezza
function p_meta_altezza = trova_punto_meta_altezza(workspace, p_alto)
    % Trova un punto a metà altezza rispetto a p_alto e sul lato opposto
    altezza_meta = p_alto(3) / 2; % Altezza metà di p_alto
    
    % Filtra i punti vicini all'altezza metà
    tolleranza = 0.05; % Definisce la tolleranza per l'altezza metà
    punti_meta = workspace(abs(workspace(:, 3) - altezza_meta) < tolleranza, :);
    
    % Trova il punto più lontano da p_alto in termini di distanza laterale (X,Y)
    distanze_laterali = sqrt((punti_meta(:, 1) - p_alto(1)).^2 + (punti_meta(:, 2) - p_alto(2)).^2);
    [~, idx_p_meta] = max(distanze_laterali); % Trova il punto più lontano lateralmente
    p_meta_altezza = punti_meta(idx_p_meta, :);
end


%% Punto Terra
function p_terra = trova_punto_terra(workspace, p_alto, p_meta_altezza)
    % Trova un punto a livello base sul lato opposto a p_alto e p_meta_altezza
    tolleranza_base = 0.05; % Tolleranza per considerare altezza zero
    
    % Filtra i punti vicini alla base (altezza ~ 0)
    punti_base = workspace(abs(workspace(:, 3)) < tolleranza_base, :);
    
    % Trova il punto più lontano da p_alto e p_meta_altezza in termini di distanza laterale
    distanze_alto = sqrt((punti_base(:, 1) - p_alto(1)).^2 + (punti_base(:, 2) - p_alto(2)).^2);
    distanze_meta = sqrt((punti_base(:, 1) - p_meta_altezza(1)).^2 + (punti_base(:, 2) - p_meta_altezza(2)).^2);
    
    % Trova il punto più lontano da entrambi (lato opposto)
    [~, idx_p_terra] = max(distanze_alto + distanze_meta); % Somma delle distanze
    p_terra = punti_base(idx_p_terra, :);
end

% %% Grafico dello spazio di lavoro
% % Visualizza il workspace
% figure;
% plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'b.', 'DisplayName', 'Punti del WS');
% hold on;
% 
% % Plot dei punti trovati con workpoint
% [pos_iniziale, p_alto, p_meta_altezza, p_terra] = workpoint(workspace);
% 
% % Evidenzia i punti specifici
% plot3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Posizione Iniziale');
% plot3(p_alto(1), p_alto(2), p_alto(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Punto Alto');
% plot3(p_meta_altezza(1), p_meta_altezza(2), p_meta_altezza(3), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Punto Meta Altezza');
% plot3(p_terra(1), p_terra(2), p_terra(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Punto Terra');
% 
% % Etichette degli assi e altri parametri grafici
% xlabel('X [m]');
% ylabel('Y [m]');
% zlabel('Z [m]');
% title('Workspace del Braccio Robotico con Punti Specifici');
% grid on;
% legend('show'); % Mostra la legenda con i nomi dei punti
% axis equal;
% hold off;


% %% Prima Versione
% function [pos_iniziale, p_alto, p_meta_altezza, p_terra] = workpoint(workspace)
%     % Funzione unica per calcolare e restituire pos_iniziale, p_alto, p_meta_altezza, p_terra
% 
%     % Seleziona una posizione iniziale casuale all'interno del workspace
%     pos_iniziale = workspace(randi(size(workspace, 1)), :);
% 
%     % Trova il punto più alto sul lato del workspace
%     p_alto = trova_punto_alto(workspace);
% 
%     % Trova il punto a metà altezza sul lato opposto di p_alto
%     p_meta_altezza = trova_punto_meta_altezza(workspace, p_alto);
% 
%     % Trova un punto a livello del terreno sul lato opposto
%     p_terra = trova_punto_terra(workspace, p_alto, p_meta_altezza);
% end
% 
% %% Punto Alto
% function p_alto = trova_punto_alto(workspace)
%     % Trova il punto più alto lateralmente nel workspace
%     altezza_massima = max(workspace(:, 3)); % Trova altezza massima
%     idx = find(workspace(:, 3) == altezza_massima); % Indici dei punti con altezza massima
%     p_candidati = workspace(idx, :); % Punti candidati
% 
%     % Filtra i candidati per posizione laterale (non vicino all'asse centrale)
%     distanza_centro = sqrt(p_candidati(:, 1).^2 + p_candidati(:, 2).^2);
%     [~, idx_p_alto] = max(distanza_centro);
%     p_alto = p_candidati(idx_p_alto, :);
% end
% 
% %% Punto Metà Altezza
% function p_meta_altezza = trova_punto_meta_altezza(workspace, p_alto)
%     % Trova un punto a metà altezza rispetto a p_alto e sul lato opposto
%     altezza_meta = p_alto(3) / 2; % Altezza metà di p_alto
% 
%     % Filtra i punti vicini all'altezza metà
%     tolleranza = 0.05; % Definisce la tolleranza per l'altezza metà
%     punti_meta = workspace(abs(workspace(:, 3) - altezza_meta) < tolleranza, :);
% 
%     % Trova il punto più lontano da p_alto (lato opposto)
%     distanze = sqrt(sum((punti_meta(:, 1:2) - p_alto(1:2)).^2, 2)); % Distanza in XY
%     [~, idx_p_meta] = max(distanze);
%     p_meta_altezza = punti_meta(idx_p_meta, :);
% end
% 
% 
% %% Punto Terra
% function p_terra = trova_punto_terra(workspace, p_alto, p_meta_altezza)
%     % Trova un punto a livello base sul lato opposto a p_alto e p_meta_altezza
%     tolleranza_base = 0.05; % Tolleranza per considerare altezza zero
% 
%     % Filtra i punti vicini alla base (altezza ~ 0)
%     punti_base = workspace(abs(workspace(:, 3)) < tolleranza_base, :);
% 
%     % Trova il punto più lontano da p_alto e p_meta_altezza
%     distanze_alto = sqrt(sum((punti_base(:, 1:2) - p_alto(1:2)).^2, 2));
%     distanze_meta = sqrt(sum((punti_base(:, 1:2) - p_meta_altezza(1:2)).^2, 2));
%     [~, idx_p_terra] = max(distanze_alto + distanze_meta); % Somma delle distanze per essere opposto a entrambi
%     p_terra = punti_base(idx_p_terra, :);
% end
