% Cleaning
c;

% Create gripper object and connect
gripper = robotiq;
gripper.Connect;
gripper.ChangeMode('Scissor');

% for i=1:255
    data = gripper.Receive;
    gripper.Send([ [ data(1,:) 0 ] ; [ data(2,:) 0 ] ; [ data(3,:) 0 ] ; [ 255 0 0 ] ])
    pause(3)
    gripper.Send([ [ data(1,:) 0 ] ; [ data(2,:) 0 ] ; [ data(3,:) 0 ] ; [ 0 0 0 ] ])
    
% end