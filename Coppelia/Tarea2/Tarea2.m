
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
    
    x_act = cart_position(1);
    y_act = cart_position(2);

    [x, y] = generadorMapa(x_act,y_act);

    if isempty(x) ||isempty(y)
        while isempty(x) ||isempty(y)
            [x, y] = generadorMapa(x_act,y_act);
        end
    end
    % Controlador PID
    Kp = 4; 
    Kd = 1; 
 
    error_dis_ant = 0;
    error_ori_ant = 0;

    figure()
    plot(x,y)
    
    % Definir la posición deseada del carrito
     % x, y coordinates
    num_positions = size(x,1);
    
    
    i = 2;
    % Comenzar la simulación
    while i <= num_positions 
    
        x_deseado = x(i,1)
        y_deseado = y(i,1)
            
        [~, cart_position] = sim.simxGetObjectPosition(clientID, pioneer, -1, sim.simx_opmode_buffer);
        x_act = cart_position(1);
        y_act = cart_position(2);
        hold on
        plot(x_act, y_act,'ob')
        
        
        [~, cart_orientation] = sim.simxGetObjectOrientation(clientID, pioneer, -1, sim.simx_opmode_buffer);
        theta_act = cart_orientation(3);
        
        error_dis = sqrt((x_deseado-x_act)^2 + (y_deseado-y_act)^2)
        error_ori = atan2(y_deseado-y_act, x_deseado-x_act) - theta_act;
        
        if error_ori > pi
            error_ori = error_ori - 2*pi;
        elseif error_ori < -pi
            error_ori = error_ori + 2*pi;
        end

        
        error_dis_der= error_dis - error_dis_ant;
        error_ori_der = error_ori - error_ori_ant;
        
        u_vel = Kp*tanh(error_dis) + Kd*tanh(error_dis_der);
        u_ori = Kp*tanh(error_ori) + Kd*tanh(error_ori_der);
        
        error_dis_ant = error_dis;
        error_ori_ant = error_ori;
        
        sim.simxSetJointTargetVelocity(clientID, left_motor, u_vel-u_ori, sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID, right_motor, u_vel+u_ori, sim.simx_opmode_blocking);
        
        

        if error_dis < 0.05
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
