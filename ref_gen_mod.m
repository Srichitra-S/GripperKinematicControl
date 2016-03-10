function [ ed , ed_dot ] = ref_gen_mod( time , Total_time )

vmA = [ 20 ; 0 ; 80 ];                    % bias      (mm)
vmB = [ 30 ; 0 ; 80 ];                   % bias      (mm)
vmC = [-30 ; 0 ; 80 ];                   % bias      (mm)

aA = [ 0 ; 0 ; 0 ];                          % amplitude (mm)
aB = [ 0 ; 0 ; 0 ];                      % amplitude (mm)
aC = [ 0 ; 0 ; 0 ];                      % amplitude (mm)

T = 10;                                  % period    (s)
wn = 2*pi/T;                             % frequency (rad/s)

eAd = vmA + aA.*sin(wn*time);
eBd = vmB + aB.*sin(wn*time);
eCd = vmC + aC.*sin(wn*time);

ed = [ eAd ; eBd ; eCd ]; % desired state vector

% Velocity feedforward terms
eAd_dot = wn*aA.*cos(wn*time);
eBd_dot = wn*aB.*cos(wn*time);
eCd_dot = wn*aC.*cos(wn*time);

ed_dot = [ eAd_dot ; eBd_dot ; eCd_dot ]; % derivative of desired state vector

end