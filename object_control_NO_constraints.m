% Run object control algorithm for the Robotiq Gripper with NO constraints.

% Cleaning and startup
% c
startup_rvc

% Setup video
% outputVideo = VideoWriter('shuttle_out.avi');
% outputVideo.FrameRate = 10;

% Operation mode
% anima_mode = 0; % do not display animation
anima_mode = 1; % display animation

% Gripper parameters
% l1 = 57; l2 = 38; l3 = 22; fe = 9/4;

% Definição do objeto
r = 30;             % raio da bola
z1 = 60;            % altura inicial da bola
% Posições dos pontos de contato (frame do objeto)
ro1 = [0 -r 0]';
ro2 = [ r*sin(pi/3) r*cos(pi/3) 0]';
ro3 = [-r*sin(pi/3) r*cos(pi/3) 0]';
% Posição e orientação inicial do objeto
po = [ 0 0 z1 ]';
Rpo = eye(3);
% Quaternion inicial
qo = Quaternion(Rpo);
eta = qo.s;
epsi = qo.v;
% Posições dos pontos de contato iniciais
p1d = ro1 + po;
p2d = ro2 + po;
p3d = ro3 + po;

% Erros iniciais
error = inf;

% Define initial conditions for simulation
thetaHand = deg2rad([ -30 ; 30 ; 30 ; 0 ; -30 ; 30 ; 30 ; -10 ; -30 ; 30 ; 30 ; -10 ]);

% Calcula os parâmetros de grasping.
GP = [ (Rpo*ro1)' ;...
       (Rpo*ro2)' ;...
       (Rpo*ro3)' ];
   
cview = [-37 30];   % initial viewpoint for animation
% cview = [0 0];
% cview = [0 90];

% anim = Animation(cview);
pause(5)

% Initialize time parameters
it = 1;
h = 0.05;
time = 0;
Total_time = 60;

% Control loop to set fingers in position
while norm(error)>=0.001
tic 

    % Cinemática direta dos dedos
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics_Free( thetaHand );
    
    % Compose states
    e1 = p1;
    e2 = p2;
    e3 = p3;
    e = [ e1 ; e2 ; e3 ]; % ---> state vector
    
    % Reference signals
    ed = [ p1d ; p2d ; p3d ];
    ed_dot = zeros(length(ed),1);
    
    % Error
    error = ed - e;
     
    % Gain definition
    sigma = 5;
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
    if (anima_mode == 1)
%         cview = Animate_Obj(thetaHand,r,po,Rpo,cview);
        anim.Animate_Obj(thetaHand,r,po,Rpo,cview);
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
%     disp('Time:');
%     disp(time);

end

% Posição relativa inicial do grasping
dp12_const = p1 - p2;
dp23_const = p2 - p3;

it_ctrl = it;

% Beginning of the control loop
while time <= Total_time
    tic
    
    % Ar matrix
    Ar = [   eye(3)   -eye(3)  zeros(3,3) ;...
           zeros(3,3)  eye(3)   -eye(3)   ];
    
    % Hand direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics_Free ( thetaHand );
    
    % Relative vectors
    dp12 = p1 - p2;
    dp23 = p2 - p3;
    
    % Estima posição do objeto (mm)
    po = mean([ p1 - Rpo*ro1 , p2 - Rpo*ro2 , p3 - Rpo*ro3 ]')';
    % Calcula os parâmetros de grasping.
    GP = [ (Rpo*ro1)' ;...
           (Rpo*ro2)' ;...
           (Rpo*ro3)' ];
    
    % Definition of the object state
    eo = po;
    % Definition of the relative state
    er = [ dp12 ; dp23 ];
    % Complete state vector
    e = [ eo ; er ]; % ---> state vector
    
    % Reference signal
    am = [ 20 20 20 ]';                        % amplitude (mm)
%     am = [0 0 30]';
    vm = [ 0 0 50 ]';                         % bias      (mm)
%     vm = [0 0 50]';
    T = 5;                                   % period (s)
    wn = 2*pi/T;                              % frequency (rad/s)
    eod = vm + [ am(1)*sin(wn*time) ; am(2)*sin(wn*time) ; am(3)*cos(wn*time) ];  % object reference
    erd = [ dp12_const ; dp23_const ];        % relative reference
    ed = [ eod ; erd ];                       % desired state vector
    
    % Velocity feedforward terms
    eod_dot = [ wn*am(1)*cos(wn*time) ; wn*am(2)*cos(wn*time) ; -wn*am(3)*sin(wn*time) ];
    erd_dot = zeros(6,1);
    ed_dot = [ eod_dot ; erd_dot ]; % derivative of desired state vector
    
    % State error
    error = ed - e;
    
    % Cartesian control signals
    sigma_obj = 5;
    sigma_rel = 10;
    K = blkdiag(sigma_obj*eye(length(eo)),sigma_rel*eye(length(er)));
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
    
    % Object jacobian matrix
    Jhand = blkdiag(J1,J2,J3);
    JhandP = blkdiag(JP1,JP2,JP3);
    
    % Object jacobian matrix
    Jo = Object_Jacobian( Jhand , GP );
    JPo = Jo(1:3,:);
    JOo = Jo(4:6,:);
    
    % State jacobian matrix
    Jstate = [    JPo    ;...
               Ar*JhandP ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
    d_qo = qo.dot(JOo*dThetaHand);
    d_eta = d_qo.s;
    d_epsi = d_qo.v;
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    eta = eta + h*d_eta;
    epsi = epsi + h*d_epsi;
    qo = Quaternion([ eta , epsi ]);
    Rpo = qo.R;         % nova matriz de rotação
    
    if (anima_mode == 1)
        % Call animation
%         cview = Animate_Obj(thetaHand,r,po,Rpo,cview);
        anim.Animate_Obj(thetaHand,r,po,Rpo,cview);
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
%     disp('Time:');
%     disp(time);
    
end