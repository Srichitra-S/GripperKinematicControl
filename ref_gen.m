function [ ed , ed_dot ] = ref_gen( time , Total_time )

limiar1 = Total_time/5;
limiar2 = 2*(Total_time/5);
limiar3 = 3*(Total_time/5);
limiar4 = 4*(Total_time/5);

if time < limiar1
    vm1 = -50;                      % bias      (mm)
    vm2 = 50;                       % bias      (mm)
    vm3 = 50;                       % bias      (mm)
    vm4 = 110;                      % bias      (mm)
elseif time < limiar2
    vm1 = -30;                      % bias      (mm)
    vm2 = 30;                       % bias      (mm)
    vm3 = 30;                       % bias      (mm)
    vm4 = 90;                       % bias      (mm)
elseif time < limiar3
    vm1 = -15;                      % bias      (mm)
    vm2 = 15;                       % bias      (mm)
    vm3 = 15;                       % bias      (mm)
    vm4 = 70;                       % bias      (mm)
elseif time < limiar4
    vm1 = -30;                      % bias      (mm)
    vm2 = 30;                       % bias      (mm)
    vm3 = 30;                       % bias      (mm)
    vm4 = 50;                       % bias      (mm)
elseif time < Total_time
    vm1 = -50;                      % bias      (mm)
    vm2 = 50;                       % bias      (mm)
    vm3 = 50;                       % bias      (mm)
    vm4 = 30;                       % bias      (mm)
end

a1 = 0;                        % amplitude (mm)
a2 = 0;                        % amplitude (mm)
a3 = 0;                        % amplitude (mm)
a4 = 0;                        % amplitude (mm)

T = 10;                         % period    (s)
wn = 2*pi/T;                    % frequency (rad/s)

e1d = +vm1 + a1*sin(wn*time);
e2d = +vm2 + a2*sin(wn*time);
e3d = +vm3 + a3*sin(wn*time);
e4d = +vm4 + a4*sin(wn*time);
ed = [ e1d ; e2d ; e3d ; e4d ]; % desired state vector

% Velocity feedforward terms
e1d_dot = wn*a1*cos(wn*time);
e2d_dot = wn*a2*cos(wn*time);
e3d_dot = wn*a3*cos(wn*time);
e4d_dot = wn*a4*cos(wn*time);
ed_dot = [ e1d_dot ; e2d_dot ; e3d_dot ; e4d_dot ]; % derivative of desired state vector

end