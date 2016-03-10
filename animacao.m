% Animação simples dos dedos da mão (usando a rotina anima.m)
clc
clear all
for i = 0:deg2rad(2):deg2rad(90)
   oa = [i i i];
   ob = [i i i];
   oc = [i i i];
   o4 = deg2rad(20);
   Animate(oa,ob,oc,o4);
   pause(0.001);
end

for i = deg2rad(90):deg2rad(-2):0
   oa = [i i i];
   ob = [i i i];
   oc = [i i i];
   o4 = deg2rad(20);
   Animate(oa,ob,oc,o4);
   pause(0.001);
end