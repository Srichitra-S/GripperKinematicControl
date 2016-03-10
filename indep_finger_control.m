% Run independent finger control for the Robotiq Gripper.

% Cleaning
% clear all
% delete(instrfind)
% clc

% Release gripper?
% gripper.Release();
% pause(1.5);

% Operation mode
op_mode = 1; % gripper mode
% op_mode = 2; % simulation mode (only animation)
% op_mode = 3; % gripper + animation mode

% Gripper parameters
l1 = 57;
l2 = 38;
l3 = 22;
fe = 9/4;

% Selection matrices
S1 = [0 1 0];
S2 = [0 1 0];
S3 = [0 1 0];
S4 = [2 0 0];   % ---> shared state

if (op_mode == 1) || (op_mode == 3)
    if ~exist('gripper','var')
        % Initialize gripper
        gripper = robotiq();
        gripper.Connect();
        gripper.ChangeMode('Individual');
    end
else
    % Define initial conditions for simulation
    thetaHand = deg2rad([ -2.4  ; -2.4  ; -2.4 ; 10 ]);
end

% Initialize time parameters
it = 1;
h = 0.1;
time = 0;
Total_time = 60;

% Beginning of the control loop
while time < Total_time 
    tic
    
    if (op_mode == 1) || (op_mode == 3)
        % Ask for gripper status
        status = gripper.Receive();
        thetaHand = byte2o( status(:,1) );
    end
    
    % Direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics ( thetaHand );
    e1 = S1*p1;
    e2 = S2*p2;
    e3 = S3*p3;
    e4 = S4*p2;                     % ---> shared state
    e = [ e1 ; e2 ; e3 ; e4 ];      % ---> state vector
    
    % References
    [ ed , ed_dot ] = ref_gen(time, Total_time);
    
    % Error
    error = ed - e;
%     error_e1 = e1d - e1;
%     error_e2 = e2d - e2;
%     error_e3 = e3d - e3;
%     error_e4 = e4d - e4;
%     error = [ error_e1 ; error_e2 ; error_e3 ; error_e4 ];
     
    % Gain definition
    switch op_mode
        case 1              % For gripper mode
        sigma = 50;
        sigma_shared = 50;
        case 2              % For simulation mode
        sigma = 10;
        sigma_shared = 10;
        case 3              % For both
        sigma = 10;
        sigma_shared = 10;
    end
    K = [ sigma*eye(3)  zeros(3,1)   ;...
           zeros(1,3)  sigma_shared  ];
       
    % Cartesian control signals
    v = K*error + ed_dot;
    
    % Angles (for jacobian matrix calculation) - all in degrees
    [ o1 , o2 , o3 ] = Finger_Angles ( thetaHand );
    
    % Geometric jacobian               % uma modificação foi feita em o1...
    [ J1, J2, J3 ] = Finger_Jacobians( o1 , o2 , o3 );
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % Calculation of the restriction Jacobian matrices
    [ Jr1, Jr2, Jc2, Jr3, Jc3 ] = Finger_Differential_Rel( thetaHand );
    
    % State jacobian for shared control
    Jstate = [  S1*JP1*Jr1       0            0            0       ;...
                   0       S2*JP2*Jr2        0       S2*JP2*Jc2   ;...
                   0            0       S3*JP3*Jr3   S3*JP3*Jc3   ;...
                   0       S4*JP2*Jr2        0       S4*JP2*Jc2   ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
%     dom = pinv(Jhand(1:3,:))*[ eye(3) zeros(3,1) ]*v;
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    if (op_mode == 1) || (op_mode == 3)
        % Conversion between angles and speeds into data to gripper
        [ pos_byte , pos_flag ] = o2byte( thetaHand );
        [ vel_byte , vel_flag ] = os2byte( abs(dThetaHand) );
        
        % Only Position
%         pose = [ pos_byte(1) 255 0 ; pos_byte(2) 255 0 ; pos_byte(3) 255 0 ; pos_byte(4) 255 0 ];

        % Only Velocity
%         index = 255*(dom(1:3) > 0);
%         index_scissor = 255*(dom(4) < 0);
%         pose = [ index(1) vel_byte(1) 0 ; index(2) vel_byte(2) 0 ; index(3) vel_byte(3) 0 ; index_scissor vel_byte(4) 0 ];

        % Position and Velocity
        pose = [ pos_byte(1) vel_byte(1) 0 ; pos_byte(2) vel_byte(2) 0 ; pos_byte(3) vel_byte(3) 0 ; pos_byte(4) vel_byte(4) 0 ];
        
        % Send data to gripper
        gripper.Send(pose);
    end
    
    if (op_mode == 2) || (op_mode == 3)
        % Call animation
        Animate(o1,o2,o3,thetaHand(4));
        % pause(0.001);
    end
    
    % Register data
    time_vec(it) = time;
    error_vec(:,it) = error;
    ref_vec(:,it) = ed;
    state_vec(:,it) = e;
    joint_ctrl_vec(:,it) = thetaHand;
    v_ctrl_vec(:,it) = v;
    dstate_vec(:,it) = dThetaHand;
    
    % Time of simulation
    it = it + 1;
    h = toc;
    time = time + h;
    
end