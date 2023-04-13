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
    positions = [.9 -2; 1.5 1.5; -1.9 .6]; % x, y coordinates
    num_positions = size(positions,1);
 
    
    t = 0;
    t_final = 10;
    
    i = 1;

    % Comenzar la simulación
    while i <= num_positions
    
        x_deseado = positions(i,1);
        y_deseado = positions(i,2);
            
        [~, cart_position] = sim.simxGetObjectPosition(clientID, pioneer, -1, sim.simx_opmode_buffer);
        x_act = cart_position(1);
        y_act = cart_position(2);
        
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
        
        error_dis_ant = error_dis;
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
