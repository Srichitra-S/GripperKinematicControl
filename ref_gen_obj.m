function [ pod , pod_dot ] = ref_gen_obj( time , Total_time )

% limiar1 = Total_time/6;
% limiar2 = 2*(Total_time/6);
% limiar3 = 3*(Total_time/6);
% limiar4 = 4*(Total_time/6);
% limiar5 = 5*(Total_time/6);

% Regulation
% if time < limiar1
%     vm = 0;                      % bias      (mm)
% elseif time < limiar2
%     vm = -30;                      % bias      (mm)
% elseif time < limiar3
%     vm = -30;                      % bias      (mm)
% elseif time < limiar4
%     vm = 30;                      % bias      (mm)
% elseif time < limiar5
%     vm = 30;                      % bias      (mm)
% elseif time < Total_time
%     vm = 0;                      % bias      (mm)
% end

% Track
vm = 0;                    % bias      (mm)

% a = 0;
a = 20;                          % amplitude (mm)

T = 10;                                  % period    (s)
wn = 2*pi/T;                             % frequency (rad/s)

% Reference
pod = vm + a*sin(wn*time);

% Velocity feedforward terms
pod_dot = wn*a*cos(wn*time);

end