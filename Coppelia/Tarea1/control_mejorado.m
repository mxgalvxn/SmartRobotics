% Inicializar la conexión con CoppeliaSim
sim=remApi('remoteApi');
sim.simxFinish(-1); % Cerrar todas las conexiones previas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

% Verificar que la conexión con CoppeliaSim fue exitosa
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
    Kp = 2; 
    Kd = 2; 
 
    error_vel_anterior = 0;
    error_ori_anterior = 0;

    
    % Definir la posición deseada del carrito
    x_deseado= .9
    y_deseado= -2
 
    
    t = 0;
    t_final = 10;
    
    % Comenzar la simulación
    while t < t_final

        [~, cart_position] = sim.simxGetObjectPosition(clientID, pioneer, -1, sim.simx_opmode_buffer);
        x_actual = cart_position(1);
        y_actual = cart_position(2);
        
        [~, cart_orientation] = sim.simxGetObjectOrientation(clientID, pioneer, -1, sim.simx_opmode_buffer);
        theta_actual = cart_orientation(3);
        
        error_vel = sqrt((x_deseado-x_actual)^2 + (y_deseado-y_actual)^2);
        error_ori = atan2(y_deseado-y_actual, x_deseado-x_actual) - theta_actual;
        
        error_vel_derivada = error_vel - error_vel_anterior;
        error_ori_derivada = error_ori - error_ori_anterior;

        u_vel = Kp*error_vel + Kd*error_vel_derivada;
        u_ori = Kp*error_ori + Kd*error_ori_derivada;
        
        error_vel_anterior = error_vel;
        error_ori_anterior = error_ori;
        
        sim.simxSetJointTargetVelocity(clientID, left_motor, u_vel-u_ori, sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID, right_motor, u_vel+u_ori, sim.simx_opmode_blocking);
        
        t = t + 0.01;
        
        pause(0.01);
    end
    
    sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_blocking);
    
    sim.simxFinish(clientID);
else
    disp('Error al conectarse con CoppeliaSim');
end
