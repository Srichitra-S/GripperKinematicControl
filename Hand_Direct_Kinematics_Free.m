% Direct kinematics of the hand. Receives the 4-vector with the four motor 
% angles of the robot hand. Returns the position vectors and rotation 
% matrices associated to each finger. All angles are in radians.

function [ p1, R1, p2, R2, p3, R3 ] = Hand_Direct_Kinematics_Free( thetaHand )

% Length of the phalanges and constant displacement of the fingers, related
% to the palm frame (both in millimeters).
l0 = 22;
l1 = 57;
l2 = 38;
l3 = 22;
pp1 = [  0   -44.5  -32 ];
pp2 = [ 36.5  44.5  -32 ];
pp3 = [-36.5  44.5  -32 ];

% Finger angles
oA1 = thetaHand(1);
oA2 = thetaHand(2);
oA3 = thetaHand(3);
oA4 = thetaHand(4);
oB1 = thetaHand(5);
oB2 = thetaHand(6);
oB3 = thetaHand(7);
oB4 = thetaHand(8);
oC1 = thetaHand(9);
oC2 = thetaHand(10);
oC3 = thetaHand(11);
oC4 = thetaHand(12);

% Sine and cosine conventions for summation
s4_1 = sin(oA4);
c4_1 = cos(oA4);
s4_2 = sin(oB4);
s4_3 = sin(oC4);
c4_2 = cos(oB4);
c4_3 = cos(oC4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s123_1 = sin(oA1+oA2+oA3);
s123_2 = sin(oB1+oB2+oB3);
s123_3 = sin(oC1+oC2+oC3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c123_1 = cos(oA1+oA2+oA3);
c123_2 = cos(oB1+oB2+oB3);
c123_3 = cos(oC1+oC2+oC3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ls1 = l1*sin(oA1)+l2*sin(oA1+oA2)+l3*s123_1;
ls2 = l1*sin(oB1)+l2*sin(oB1+oB2)+l3*s123_2;
ls3 = l1*sin(oC1)+l2*sin(oC1+oC2)+l3*s123_3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lc1 = l1*cos(oA1)+l2*cos(oA1+oA2)+l3*c123_1;
lc2 = l1*cos(oB1)+l2*cos(oB1+oB2)+l3*c123_2;
lc3 = l1*cos(oC1)+l2*cos(oC1+oC2)+l3*c123_3;

% Position vector for each finger (with respect to the palm frame)
p1 = [ (lc1 + l0)*s4_1 + pp1(1) ; ls1 + pp1(2) ; (lc1+l0)*c4_1 + pp1(3) ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2 = [  (lc2 + l0)*s4_2 + pp2(1)  ;  pp2(2) - ls2  ;   (lc2 + l0)*c4_2 + pp2(3) ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3 = [  -(lc3 + l0)*s4_3 + pp3(1)  ;  pp3(2) - ls3  ;  (lc3 + l0)*c4_3 + pp3(3) ]; %%%---> verificar sinais!!

% Rotation matrices for each finger
R1 = [   c4_1  -s123_1*s4_1   c123_1*s4_1   ;...
           0      c123_1        s123_1      ;...
        -s4_1  -s123_1*c4_1   c123_1*c4_1   ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R2 = [   c4_2   s123_2*s4_2   c123_2*s4_2   ;...
           0      c123_2       -s123_2      ;...
        -s4_2   s123_2*c4_2   c123_2*c4_2   ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R3 = [   c4_3  -s123_3*s4_3  -c123_3*s4_3   ;...
           0      c123_3       -s123_3      ;...
         s4_3   s123_3*c4_3   c123_3*c4_3   ];

end