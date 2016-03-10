% Run independent finger control for a 12 DoF robot hand, similar to the
% Robotiq Gripper.

% Cleaning
c

% Gripper parameters
l1 = 57;
l2 = 38;
l3 = 22;
fe = 9/4;

% Define initial conditions for simulation
thetaHand = deg2rad([ -30 ; 30 ; 30 ; 0 ; -30 ; 30 ; 30 ; -10 ; -30 ; 30 ; 30 ; -10 ]);

cview = [-37 30];   % initial viewpoint for animation
% cview = [0 0];   % initial viewpoint for animation
% [~,~,gui] = Gripper_Simulation;

% Initialize time parameters
it = 1;
h = 0.1;
time = 0;
Total_time = 60;

% Beginning of the control loop
while time < Total_time 
    tic
    
    % Direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics_Free ( thetaHand );
    e1 = p1;
    e2 = p2;
    e3 = p3;
    e = [ e1 ; e2 ; e3 ];      % ---> state vector
    
    % References
    [ ed , ed_dot ] = ref_gen_mod(time, Total_time);
    
    % Error
    error = ed - e;
     
    % Gain definition
    sigma = 1;
    K = sigma*eye(length(e));

    % Cartesian control signals
    v = K*error + ed_dot;
    
    % Geometric jacobian
    o1 = thetaHand(1:4);
    o2 = thetaHand(5:8);
    o3 = thetaHand(9:12);
    
    [ J1, J2, J3 ] = Finger_Jacobians_Free( o1 , o2 , o3 );
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % State jacobian for shared control
    Jstate = blkdiag(JP1,JP2,JP3);
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = pinv(Jstate)*v;
%     dom = pinv(Jhand(1:3,:))*[ eye(3) zeros(3,1) ]*v;
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    % Call animation
    cview = Animate2(thetaHand,cview);
    % pause(0.001);
    
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