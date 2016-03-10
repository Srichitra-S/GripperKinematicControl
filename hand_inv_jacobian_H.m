function [ dqa ] = hand_inv_jacobian_H( in )
% Calcula a jacobiana inversa do dedo B. Pode-se dizer que é a parte central do
% algoritmo. Fornece a derivada do vetor de ângulos das juntas atuadas (dqa - 1x2).
% Em graus/segundo.

% Parâmetros da garra necessários ao algoritmo.
l1 = 5.7;
l2 = 3.8;
l3 = 2.2;
l4 = 2.2;
fe = 9/4;

% Significado das entradas.
dy = in(1);
omA = in(2);
omB = in(3);
omC = in(4);
os = in(5); % não utilizado
yobj = in(6);

% Parâmetros do objeto.
d = 0;
pax = 0;
pay = -yobj;
paz = 0;
pbx = d/2;
pby = yobj;
pbz = 0;
pcx = -d/2;
pcy = yobj;
pcz = 0;

% Calcula os ângulos pelo modelo empírico, com o auxílio de angulos.m.
% angulos.m fornece a saída em graus, portanto convertemos para radianos
% antes do cálculo dos cossenos.
o = [omA omB omC];
% o = [0 omB 0];
[outilA,outilB,outilC] = angulos(o);
o1A = degtorad(outilA(1));
o2A = degtorad(outilA(2));
o3A = degtorad(outilA(3));
o1B = degtorad(outilB(1));
o2B = degtorad(outilB(2));
o3B = degtorad(outilB(3));
o1C = degtorad(outilC(1));
o2C = degtorad(outilC(2));
o3C = degtorad(outilC(3));
osB = degtorad(os);
osC = degtorad(-os);

% Calcula os param. do modelo empírico.
[delta1A,delta2A,delta3A] = delta(omA);
[delta1B,delta2B,delta3B] = delta(omB);
[delta1C,delta2C,delta3C] = delta(omC);

% O algoritmo, definido pela matriz jacobiana em si. Utiliza os param.
% "deltas" e os ângulos o1, o2 e o3, inferidos a partir dos encoders dos
% motores.
% domA = (((delta1A*(l1*cos(o1A)+l2*cos(o1A+o2A)+l3*cos(o1A+o2A+o3A))+fe*(1-delta2A)*(l2*cos(o1A+o2A)+l3*cos(o1A+o2A+o3A))-delta1A*delta3A*l3*cos(o1A+o2A+o3A))^-1)*dyeA);
% domB = (((-delta1B*(l1*cos(o1B)+l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B))-fe*(1-delta2B)*(l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B))+delta1B*delta3B*l3*cos(o1B+o2B+o3B))^-1)*dyeB);
% domC = (((-delta1C*(l1*cos(o1C)+l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C))-fe*(1-delta2C)*(l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C))+delta1C*delta3C*l3*cos(o1C+o2C+o3C))^-1)*dyeC);
% dosB = dphye;

Ja11 = 0;
Ja21 = (delta1A*(l1*cos(o1A)+l2*cos(o1A+o2A)+l3*cos(o1A+o2A+o3A))+fe*(1-delta2A)*(l2*cos(o1A+o2A)+l3*cos(o1A+o2A+o3A))-delta1A*delta3A*l3*cos(o1A+o2A+o3A));
Ja31 = (-delta1A*(l1*sin(o1A)+l2*sin(o1A+o2A)+l3*sin(o1A+o2A+o3A))-fe*(1-delta2A)*(l2*sin(o1A+o2A)+l3*sin(o1A+o2A+o3A))+delta1A*delta3A*l3*sin(o1A+o2A+o3A));
Ja41 = delta1A+fe*(1-delta2A)-delta1A*delta3A;
Ja51 = 0;
Ja61 = 0;

Ja = [Ja11 ; Ja21 ; Ja31 ; Ja41 ; Ja51 ; Ja61];

Jb11 = (-delta1B*(l1*sin(o1B)+l2*sin(o1B+o2B)+l3*sin(o1B+o2B+o3B))*sin(osB)-fe*(1-delta2B)*(l2*sin(o1B+o2B)+l3*sin(o1B+o2B+o3B))*sin(osB)+delta1B*delta3B*l3*sin(o1B+o2B+o3B)*sin(osB));
Jb12 = (l1*cos(o1B)+l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B)+l4)*cos(osB);
Jb21 = (-delta1B*(l1*cos(o1B)+l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B))-fe*(1-delta2B)*(l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B))+delta1B*delta3B*l3*cos(o1B+o2B+o3B));
Jb22 = 0;
Jb31 = (-delta1B*(l1*sin(o1B)+l2*sin(o1B+o2B)+l3*sin(o1B+o2B+o3B))*cos(osB)-fe*(1-delta2B)*(l2*sin(o1B+o2B)+l3*sin(o1B+o2B+o3B))*cos(osB)+delta1B*delta3B*l3*sin(o1B+o2B+o3B)*cos(osB));
Jb32 = -(l1*cos(o1B)+l2*cos(o1B+o2B)+l3*cos(o1B+o2B+o3B)+l4)*sin(osB);
Jb41 = delta1B*cos(osB)+fe*(1-delta2B)*cos(osB)-delta1B*delta3B*cos(osB);
Jb42 = 0;
Jb51 = 0;
Jb52 = 1;
Jb61 = -delta1B*sin(osB)-fe*(1-delta2B)*sin(osB)+delta1B*delta3B*sin(osB);
Jb62 = 0;

Jb = [Jb11 Jb12 ; Jb21 Jb22 ; Jb31 Jb32 ; Jb41 Jb42 ; Jb51 Jb52 ; Jb61 Jb62 ];

Jc11 = (delta1C*(l1*sin(o1C)+l2*sin(o1C+o2C)+l3*sin(o1C+o2C+o3C))*sin(osC)+fe*(1-delta2C)*(l2*sin(o1C+o2C)+l3*sin(o1C+o2C+o3C))*sin(osC)-delta1C*delta3C*l3*sin(o1C+o2C+o3C)*sin(osC));
Jc12 = (l1*cos(o1C)+l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C)+l4)*cos(osC);
Jc21 = (-delta1C*(l1*cos(o1C)+l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C))-fe*(1-delta2C)*(l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C))+delta1C*delta3C*l3*cos(o1C+o2C+o3C));
Jc22 = 0;
Jc31 = (-delta1C*(l1*sin(o1C)+l2*sin(o1C+o2C)+l3*sin(o1C+o2C+o3C))*cos(osC)-fe*(1-delta2C)*(l2*sin(o1C+o2C)+l3*sin(o1C+o2C+o3C))*cos(osC)+delta1C*delta3C*l3*sin(o1C+o2C+o3C)*cos(osC));
Jc32 = (l1*cos(o1C)+l2*cos(o1C+o2C)+l3*cos(o1C+o2C+o3C)+l4)*sin(osC);
Jc41 = delta1C*cos(osC)+fe*(1-delta2C)*cos(osC)-delta1C*delta3C*cos(osC);
Jc42 = 0;
Jc51 = 0;
Jc52 = -1;
Jc61 = delta1C*sin(osC)+fe*(1-delta2C)*sin(osC)-delta1C*delta3C*sin(osC);
Jc62 = 0;

Jc = [Jc11 Jc12 ; Jc21 Jc22 ; Jc31 Jc32 ; Jc41 Jc42 ; Jc51 Jc52 ; Jc61 Jc62 ];

J = [Ja  zeros(6,3) ; zeros(6,1) Jb(:,1) zeros(6,1) Jb(:,2) ; zeros(6,2) Jc(:,1) Jc(:,2) ];

Sa = [0 -paz pay ; paz 0 -pax ; -pay pax 0];
Aa = [eye(3) -Sa ; zeros(3,3) eye(3)];
Sb = [0 -pbz pby ; pbz 0 -pbx ; -pby pbx 0];
Ab = [eye(3) -Sb ; zeros(3,3) eye(3)];
Sc = [0 -pcz pcy ; pcz 0 -pcx ; -pcy pcx 0];
Ac = [eye(3) -Sc ; zeros(3,3) eye(3)];

A = [Aa ; Ab ; Ac];
Atil = [Ab -Aa zeros(6,6)]; % aniquiladora de A
S = [0 1 0 0 0 0]; % matriz seletora de y
Ha = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)]; % modelos de contato
Hb = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)];
Hc = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)];
H = [Ha zeros(6,12) ; zeros(6,6) Hb zeros(6,6) ; zeros(6,12) Hc];
Jc = Atil*[J H];
Jco = Jc(:,[1 2 3 4]);
Jcw = Jc(:,[5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22]);
Jm = pinv(A)*[J H];
Jmo = Jm(:,[1 2 3 4]);
Jmw = Jm(:,[5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22]);
% Jhand = S*(Jmo-Jmw*pinv(Jcw)*Jco);
Jhand_full = (Jmo-Jmw*pinv(Jcw)*Jco);
Jhand = S*Jhand_full;
hand_inv_jacobian = pinv(Jhand);

dqa = hand_inv_jacobian*dy;

end