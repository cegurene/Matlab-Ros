%% Leer sensores


% Obtener la ultima medida del laser
msg_laser= sub_laser.LatestMessage;

% Representación grafica
rosPlot(msg_laser);
