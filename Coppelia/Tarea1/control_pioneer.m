%%Inicio de conexion

sim=remApi('remoteApi'); % usando el prototipo de función (remoteApiProto.m) 
sim.simxFinish(-1); % Cerrar las conexiones anteriores en caso de que exista una 
clientID=sim.simxStart('127.0.0.1',19999, true, true, 5000,5);
    
%% Verificación de la conexión 

if (clientID>-1) 
    disp("Conexión con Coppelia iniciada");
  
    u =3 ;
    %%Codigo de control
    %%Preparación
    [returnCode, left_motor] = sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);

    [returnCode, right_motor] = sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);

    %Acciones 
      
    [~, pioneer_block] = sim.simxGetObjectHandle (clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking);
    [returnCode, target_block] = sim.simxGetObjectHandle(clientID, 'Cuboid', sim.simx_opmode_blocking);

    %Seguir posicion
    [~, position_pioneer] = sim.simxGetObjectPosition (clientID, pioneer_block, -1, sim.simx_opmode_streaming);
    [~, orientation_pioneer] = sim.simxGetObjectOrientation (clientID, pioneer_block, -1, sim.simx_opmode_streaming);
    [~, target] = sim.simxGetObjectPosition (clientID, target_block, -1, sim.simx_opmode_streaming);
    

    %parametros de control
    x = position_pioneer(1); % x-position (m)
    y = position_pioneer(2); % y-position (m)
    theta = orientation_pioneer(u);
    theta = rad2deg(theta);

    
  
    
    xd = target(1);
    yd = target(2);

    kpr = 1;
    kpt = 1;

    for i = 1:200
 
        [~, position_pioneer] = sim.simxGetObjectPosition (clientID, pioneer_block, -1, sim.simx_opmode_buffer);
        [~, orientation_pioneer] = sim.simxGetObjectOrientation (clientID, pioneer_block, -1, sim.simx_opmode_buffer);
        [~, target] = sim.simxGetObjectPosition (clientID, target_block, -1, sim.simx_opmode_buffer);
        %x_info = sprintf('La diferencia entre ambos objetos es: (%f , %f)\n', position_pioneer(1) - target(1), position_pioneer(2) - target(2));
        %disp(x_info);
    
        x = position_pioneer(1); % x-position (m)
        y = position_pioneer(2);
        theta = orientation_pioneer(u);
        theta = rad2deg(theta);
        
        thetad = atan2((yd-y),(xd-x));
        

        d = sqrt((xd-x)^2 + (yd-y)^2);
   
        thetae = (theta-thetad);
        disp(thetae)
        if thetae > pi
            thetae = thetae - 2*pi;
        elseif thetae < -pi
            thetae = thetae + 2*pi;
        end

        w = -kpr*thetae;
        v = kpt*d;
        vx = v;
        vy = v;
        disp(theta)
        
        relativeTo = sim.sim_handle_parent; % Establecer el objeto padre como referencia
        eulerAngles = [0, 0, w]; % Rotar 90 grados alrededor del eje Z

        [returnCode] = sim.simxSetJointTargetVelocity(clientID, left_motor, vy,...
        sim.simx_opmode_blocking);

        [returnCode] = sim.simxSetJointTargetVelocity(clientID, right_motor, vx,...
        sim.simx_opmode_blocking);
         relativeTo = -1;
         [returnCode] = sim.simxSetObjectOrientation(clientID, pioneer_block, ...
        relativeTo, eulerAngles, sim.simx_opmode_blocking);


        pause(0.1)

    end
    
%     [returnCode] = sim.simxSetJointTargetVelocity(clientID, left_motor,0,...
%         sim.simx_opmode_blocking);
    disp("Conexión con Coppelia Terminada");
    sim.simxFinish(clientID);

end

sim.delete(); % Llamar al destructor!
