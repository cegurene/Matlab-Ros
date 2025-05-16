%% Funcion paredes
function codigo = detectar_paredes()
    sonar0=rossubscriber('/robot0/sonar_0'); % Subscripción al sonar delantero
    sonarIzq=rossubscriber('/robot0/sonar_1'); % Subscripción al sonar1
    sonarDer=rossubscriber('/robot0/sonar_2'); % Subscripción al sonar2
    sonarAtras1=rossubscriber('/robot0/sonar_3'); % Subscripción al sonar3
    sonarAtras2=rossubscriber('/robot0/sonar_4'); % Subscripción al sonar4
    pause(2)

    % Definir umbral de detección de pared (por ejemplo, 2 metros)
    umbral = 2; 
    
    % Inicializar código binario de paredes
    codigo = [0 0 0 0];
    
    % Verificar detección de paredes
    if sonar0.LatestMessage.Range_ < umbral
        codigo(1) = 1; % Bit más significativo para la pared frontal
    end
    if sonarAtras1.LatestMessage.Range_ < umbral & sonarAtras2.LatestMessage.Range_ < umbral
        codigo(3) = 1; % Pared trasera
    end
    if sonarIzq.LatestMessage.Range_ < umbral
        codigo(2) = 1; % Pared izquierda
    end
    if sonarDer.LatestMessage.Range_ < umbral
        codigo(4) = 1; % Pared derecha
    end
    
end


%% Ejemplo
codigo = detectar_paredes();
codigo = sprintf('%d%d%d%d', codigo);
% 1er digito: pared delantera
% 2º digito: pared izquierda
% 3º digito: pared trasera
% 4º digito: pared derecha

disp(['Código de paredes detectadas: ', num2str(codigo)]);
