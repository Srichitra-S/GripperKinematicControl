% Abrir ligeiramente o scissor.
% Rode o script sem o objeto.

gripper.Release();
status = gripper.Receive();
pose = [status(1,1) 0 0 ; status(2,1) 0 0 ; status(3,1) 0 0 ; status(4,1)-15 0 0 ];
gripper.Send(pose);

status = gripper.Receive();
thetaHand = byte2o( status(:,1) );

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