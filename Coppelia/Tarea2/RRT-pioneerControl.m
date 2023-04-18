close all
clc

if exist('BinaryMap','var') == 0 
    BinaryMap = makemap(500)
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
text = ['tiempo ejecucion de RRT: ',num2str(time_rrt)];
disp(text);

% figure()
% show(map)
% hold on
% plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
% plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path

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
text = ['tiempo ejecucion de RRT*: ',num2str(time_rrt)];
disp(text);

figure()
show(map)
hold on
plot(solnInfo_star.TreeData(:,1),solnInfo_star.TreeData(:,2),'.-'); % tree expansion
plot(pthObj_star.States(:,1),pthObj_star.States(:,2),'r-','LineWidth',2) % draw path



x = pthObj_star.States(:,1)
y = pthObj_star.States(:,2)


disp("Iniciando carrito");
% Inicializar la conexión con CoppeliaSim
sim=remApi('remoteApi');
sim.simxFinish(-1); % Cerrar todas las conexiones previas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID>-1)
    disp('Conexión establecida con CoppeliaSim');
    
    % Definir los handles de los motores y el carrito en CoppeliaSim
    [returnCode, left_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [returnCode, right_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
    [~, pioneer] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking);
    [returnCode, target_block] = sim.simxGetObjectHandle(clientID, 'Cuboid0', sim.simx_opmode_blocking);
  

    [~, cart_position] = sim.simxGetObjectPosition (clientID, pioneer, -1, sim.simx_opmode_streaming);
    [~, cart_orientation] = sim.simxGetObjectOrientation(clientID, pioneer, -1, sim.simx_opmode_streaming);
    [~, target] = sim.simxGetObjectPosition (clientID, target_block, -1, sim.simx_opmode_streaming);

    % Controlador PID
    Kp = 1.5; 
    Kd = 1; 
 
    error_dis_ant = 0;
    error_ori_ant = 0;
    
    
    % Definir la posición deseada del carrito
     % x, y coordinates
    num_positions = size(x,1);
 
    
    t = 0;
    t_final = 10;
    
    i = 1;
    hold on;
    % Comenzar la simulación
    while i <= num_positions
    
        x_deseado = pthObj_star.States(i,1);
        y_deseado = pthObj_star.States(i,2);
            
        [~, cart_position] = sim.simxGetObjectPosition(clientID, pioneer, -1, sim.simx_opmode_buffer);
        x_act = cart_position(1);
        y_act = cart_position(2);

        plot(x_act, y_act,'o');
        [~, cart_orientation] = sim.simxGetObjectOrientation(clientID, pioneer, -1, sim.simx_opmode_buffer);
        theta_act = cart_orientation(3);
        
        error_dis = sqrt((x_deseado-x_act)^2 + (y_deseado-y_act)^2);
        error_ori = atan2(y_deseado-y_act, x_deseado-x_act) - theta_act;
        
        if error_ori > pi
            error_ori = error_ori - 2*pi;
        elseif error_ori < -pi
            error_ori = error_ori + 2*pi;
        end

        
        error_dis_der= error_dis - error_dis_ant;
        error_ori_der = error_ori - error_ori_ant;

        u_vel = Kp*error_dis + Kd*error_dis_der;
        u_ori = Kp*error_ori + Kd*error_ori_der;
        
        error_dis_ant = error_dis
        error_ori_ant = error_ori;
        
        sim.simxSetJointTargetVelocity(clientID, left_motor, u_vel-u_ori, sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID, right_motor, u_vel+u_ori, sim.simx_opmode_blocking);
        
        t = t + 0.01;
        
        if error_dis < 0.02
        i = i+1;
        pause(1); % wait for 1 second before moving to the next reference
        end

    end
    
    sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_blocking);
    
    sim.simxFinish(clientID);
else
    disp('Error al conectarse con CoppeliaSim');
end
