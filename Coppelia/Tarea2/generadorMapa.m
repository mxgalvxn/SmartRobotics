function [x, y] = generadorMapa() 
close all
clc

if exist('BinaryMap','var') == 0 
    BinaryMap = makemap(500);
end

% makemap:
%   left button, click and drag to create a rectangle
%   or type the following letters in the figure window:
%   p - draw polygon
%   c - draw circle
%   e - erase map
%   u - undo last action
%   q - leave editing mode

% figure()
%map = binaryOccupancyMap(sourcemap,resolution)
map = binaryOccupancyMap(rot90(transpose(BinaryMap)),1000/10);
% show(map)

% Creamos un espacio de estados 2-dimensional
ss = stateSpaceSE2;
% Creamos un validador de estado a partir de SS
sv = validatorOccupancyMap(ss);
% Cargamos el mapa dibujado
sv.Map = map;
% Definimos una distancia de validación
sv.ValidationDistance = 0.01;
% Hacemos que las dimensiones del SS sean las mismas del mapa
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

planner = plannerRRT(ss,sv);
planner.MaxIterations = 200000;
planner.MaxConnectionDistance = 0.5;

startLocation = [0 0 0];
endLocation = [5 5 0];

tic
[pthObj,solnInfo] = plan(planner,startLocation,endLocation);
time_rrt = toc;
text = ['Tiempo de ejecucion de RRT: ',num2str(time_rrt)];
disp(text);

%% Ahora para RRTStar

planner_star = plannerRRTStar(ss,sv);
% Seguimos afinando el camino
planner_star.ContinueAfterGoalReached = true;
% Ponemos un límite para no irnos al infinito
planner_star.MaxIterations = 2000;
planner_star.MaxConnectionDistance = 0.5;

tic
[pthObj_star,solnInfo_star] = plan(planner_star,startLocation,endLocation);
time_rrt = toc;
text = ['Tiempo de ejecucion de RRT*: ',num2str(time_rrt)];
disp(text);

figure()
show(map)
hold on
plot(solnInfo_star.TreeData(:,1),solnInfo_star.TreeData(:,2),'.-'); % tree expansion
plot(pthObj_star.States(:,1),pthObj_star.States(:,2),'r-','LineWidth',2) % draw path

x = pthObj_star.States(:,1);
y = pthObj_star.States(:,2);

end