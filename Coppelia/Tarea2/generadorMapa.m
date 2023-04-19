function [x, y] = generadorMapa(x_carrito,y_carrito) 
close all
clc

load('binaryMap.mat')
% makemap:
%   left button, click and drag to create a rectangle
%   or type the following letters in the figure window:
%   p - draw polygonSSSS
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


startLocation = [.5 .5 0];
endLocation = [4.5 4.5 0];


%% Ahora para RRTStar

planner_star = plannerRRTStar(ss,sv);
% Seguimos afinando el camino
planner_star.ContinueAfterGoalReached = true;
% Ponemos un límite para no irnos al infinito
planner_star.MaxIterations = 2000;
planner_star.MaxConnectionDistance = 0.5;

tic
[pthObj_star,solnInfo_star] = plan(planner_star,startLocation,endLocation);
% text = ['Tiempo de ejecucion de RRT*: ',num2str(time_rrt)];
disp('Ruta no encontrada');

figure()
show(map)
hold on
plot(solnInfo_star.TreeData(:,1),solnInfo_star.TreeData(:,2),'.-'); % tree expansion
plot(pthObj_star.States(:,1),pthObj_star.States(:,2),'r-','LineWidth',2) % draw path

x = pthObj_star.States(:,1);
y = pthObj_star.States(:,2);

end
