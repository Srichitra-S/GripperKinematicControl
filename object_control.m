% Run object control algorithm for the Robotiq Gripper.

% % Cleaning
% clear all
% delete(instrfind)
% clc

% Release gripper?
% gripper.Release();
% pause(1.5);

% Operation mode
% op_mode = 1; % gripper mode
op_mode = 2; % simulation mode (only animation)
% op_mode = 3; % gripper + animation mode

% Catch mode
catch_mode = 0; % do not catch object
% catch_mode = 1; % catch object


% Gripper parameters
l1 = 57;
l2 = 38;
l3 = 22;
fe = 9/4;

% Grasping selection matrix
So = [ 0 1 0 ];

if (op_mode == 1) || (op_mode == 3)
    if ~exist('gripper','var')
        % Initialize gripper
        gripper = robotiq();
        gripper.Connect();
        gripper.ChangeMode('Individual');
    end
    if catch_mode == 1
        % Obtain grasping parameters
        [ GP , DELTA ] = gripper.Detect();
        pause(2)
    end
else
    % Define initial conditions for simulation
    thetaHand = deg2rad([ 30  ; 30 ; 30 ; -10 ]);
    % Cinemática direta dos dedos
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics(thetaHand);
    obj_dim = abs(p2(2)-p1(2)); % largura do objeto em y (mm)
    % Calcula os parâmetros de grasping.
    GP = [   0   -obj_dim/2  0 ;...
           p2(1)  obj_dim/2  0 ;...
           p3(1)  obj_dim/2  0 ];
    % Calcula a separação dos dedos
    DELTA = [ (p1-p2)' ;...
              (p2-p3)' ;...
              (p3-p1)' ];
end

% Initialize time parameters
it = 1;
h = 0.1;
time = 0;

% Beginning of the control loop
while true
    tic
    
    if (op_mode == 1) || (op_mode == 3)
        % Ask for gripper status
        status = gripper.Receive();
        thetaHand = byte2o( status(:,1) );
    end
    
    % Hand direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics ( thetaHand );
    % Distance between the fingers
    dp12 = p1-p2;
    dp23 = p2-p3;
    dp31 = p3-p1;
    % Estimate of object position
    po = mean( [ (p1(2)-GP(1,2)) ; (p2(2)-GP(2,2)) ; (p3(2)-GP(3,2)) ] );
    
    % Definition of the object state
    eo = po;
    % Definition of the relative state
    er = [ sqrt(dp12'*dp12) ; sqrt(dp23'*dp23) ; sqrt(dp31'*dp31) ];
    % Complete state vector
    e = [ eo ; er ]; % ---> state vector
    
    % References
    am = 20;                                  % amplitude (mm)
    vm = 0;                                   % bias      (mm)
    T = 10;                                   % period    (s)
    wn = 2*pi/T;                              % frequency (rad/s)
    eod = vm + am*sin(wn*time);               % object reference
    erd = [ sqrt(DELTA(1,:)*DELTA(1,:)') ; sqrt(DELTA(2,:)*DELTA(2,:)') ; sqrt(DELTA(3,:)*DELTA(3,:)') ]; % relative reference
    ed = [ eod ; erd ];                       % desired state vector
    
    % Velocity feedforward terms
    eod_dot = wn*am*cos(wn*time);
    erd_dot = zeros(3,1);
    ed_dot = [ eod_dot ; erd_dot ]; % derivative of desired state vector
    
    % State error
    error = ed - e;
    
    % Cartesian control signals
    sigma_obj = 10;
    sigma_rel = 10;
    K = [  sigma_obj      zeros(1,3)    ;...
           zeros(3,1)  sigma_rel*eye(3) ];
    v = K*error + ed_dot;
    
    % Angles (for jacobian matrix calculation) - all in degrees
    [ o1 , o2 , o3 ] = Finger_Angles ( thetaHand );
    
    % Geometric jacobian of each finger
    [ J1, J2, J3 ] = Finger_Jacobians( o1 , o2 , o3 );
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % Calculation of the restriction Jacobian matrices
    [ Jr1, Jr2, Jc2, Jr3, Jc3 ] = Finger_Differential_Rel( thetaHand );
    
    % Hand jacobian matrix
    Jhand = [               J1*Jr1                  zeros(size(J1*Jr1,1),size(J2*Jr2,2))  zeros(size(J1*Jr1,1),size(J3*Jr3,2))  zeros(size(J1*Jr1,1),size(J3*Jr3,2)) ;...
              zeros(size(J2*Jr2,1),size(J1*Jr1,2))                 J2*Jr2                 zeros(size(J2*Jr2,1),size(J3*Jr3,2))                  J2*Jc2               ;...
              zeros(size(J3*Jr3,1),size(J1*Jr1,2))  zeros(size(J3*Jr3,1),size(J2*Jr2,2))               J3*Jr3                                   J3*Jc3               ];
    
    % Object jacobian matrix
    Jo = Object_Jacobian( Jhand , GP );
    JPo = Jo(1:3,:);
    
    % Relative hand jacobian
    Jhr = [              JP1*Jr1                                   -JP2*Jr2               zeros(size(JP2*Jr2,1),size(JP3*Jr3,2))      -JP2*Jc2       ;...
            zeros(size(JP2*Jr2,1),size(JP1*Jr1,2))                  JP2*Jr2                            -JP3*Jr3                   JP2*Jc2 - JP3*Jc3  ;...
                        -JP1*Jr1                   zeros(size(JP1*Jr1,1),size(JP2*Jr2,2))               JP3*Jr3                        JP3*Jc3       ];
    
    % Delta diagonal matrix
    D = [ (dp12')/er(1) zeros(1,6) ; zeros(1,3) (dp23')/er(2) zeros(1,3) ; zeros(1,6) (dp31')/er(3) ];
    
    % State jacobian matrix
    Jstate = [ So*JPo ;...
               D*Jhr  ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
%     dom = pinv(Jstate(1,:))*v(1);
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    if (op_mode == 1) || (op_mode == 3)
        % Send new position to gripper
        [ byte , flag ] = o2byte( thetaHand );
        pose = [ byte(1) 255 0 ; byte(2) 255 0 ; byte(3) 255 0 ; byte(4) 255 0 ];
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
    clc
    disp('Time:');
    disp(time);
    
end