close all
clear all

%% Runge-Kutta 4º order

r1 = 2;
r2 = 1;
mass1 = 3;
mass2 = 3;
g= 9.81;
time = 40;
delta_t = 0.02;

ang1 = deg2rad(90);
ang2 = deg2rad(0);
vel1 = 2;
vel2 = 0;

r_RK4 = [];


for t = 0:delta_t:time
    [ang1,ang2,vel1,vel2] = metodo_de_Runge_Kutta(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t);
    em = mechanical_energy(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    r_RK4 = [r_RK4;em];
end

%% Euler

ang1 = deg2rad(90);
ang2 = deg2rad(0);
vel1 = 2;
vel2 = 0;

r_Euler = [];

for t = 0:delta_t:time
    [ang1,ang2,vel1,vel2] = Euler_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t);
    em = mechanical_energy(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    r_Euler = [r_Euler;em];
end


plot(0:delta_t:time,r_RK4,'b-')
hold on
plot(0:delta_t:time,r_Euler,'r-')
xlabel("time")
ylabel("Mechanical Energy")
legend("RK4","Euler")

%%


function f = aceleration1(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    d1 = -g*(2*mass1 + mass2) * sin(ang1) - mass2*g*sin(ang1 - 2*ang2) - 2*sin(ang1 - ang2)*mass2*((vel2^2)*r2 + (vel1^2)*r1*cos(ang1- ang2));
    d2 = r1*(2*mass1+mass2-mass2*cos(2*ang1-2*ang2));
    f = (d1)/d2;
end


function f = aceleration2(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    d1 = 2*sin(ang1 - ang2)*((vel1^2)*r1*(mass1 + mass2) + g*(mass1 + mass2)*cos(ang1) + (vel2^2)*r2*mass2*cos(ang1 - ang2));
    d2 = r2*(2*mass1+mass2-mass2*cos(2*ang1-2*ang2));
    f = (d1)/d2;
end


function [angle1,angle2,velocity1,velocity2] = metodo_de_Runge_Kutta(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t)
    
    %valores iniciais
    
    v1=vel1;
    v2=vel2;
    a1=ang1;
    a2=ang2;
    ac1=aceleration1(g,mass1,mass2,a1,a2,v1,v2,r1,r2);
    ac2=aceleration2(g,mass1,mass2,a1,a2,v1,v2,r1,r2);
    
    %k1

    v1_1=v1+(delta_t*ac1)/2;
    v2_1=v2+(delta_t*ac2)/2;
    a1_1=a1+(delta_t*v1_1)/2;
    a2_1=a2+(delta_t*v2_1)/2;
    acl_1=aceleration1(g,mass1,mass2,a1_1,a2_1,v1_1,v2_1,r1,r2);
    ac2_1=aceleration2(g,mass1,mass2,a1_1,a2_1,v1_1,v2_1,r1,r2);
    
    %k2

    vl_2=v1_1+(delta_t*acl_1)/2;
    v2_2=v2_1+(delta_t*ac2_1)/2;
    a1_2=a1_1+(delta_t*vl_2)/2;
    a2_2=a2_1+(delta_t*v2_2)/2;
    acl_2=aceleration1(g,mass1,mass2,a1_2,a2_2,vl_2,v2_2,r1,r2);
    ac2_2=aceleration2(g,mass1,mass2,a1_2,a2_2,vl_2,v2_2,r1,r2);
    
    %k3

    vl_3=v1_1+(delta_t*acl_2);
    v2_3=v2_1+(delta_t*ac2_2);
    a1_3=a1_1+(delta_t*vl_3);
    a2_3=a2_1+(delta_t*v2_3);
    acl_3=aceleration1(g,mass1,mass2,a1_3,a2_3,vl_3,v2_3,r1,r2);
    ac2_3=aceleration2(g,mass1,mass2,a1_3,a2_3,vl_3,v2_3,r1,r2);
    
    %k4
    
    velocity1=v1+(delta_t*(ac1+2*acl_1+2*acl_2+acl_3))/6;
    angle1=a1+(delta_t*(v1+2*v1_1+2*vl_2+vl_3))/6;
    
    velocity2=v2+(delta_t*(ac2+2*ac2_1+2*ac2_2+ac2_3))/6;
    angle2=a2+(delta_t*(v2+2*v2_1+2*v2_2+v2_3))/6;
end

function [ang1,ang2,vel1,vel2] = Euler_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t)

    ac1=aceleration1(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    ac2=aceleration2(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    
    vel1 = vel1+ac1*delta_t;
    vel2 = vel2+ac2*delta_t;
    
    ang1 = ang1+vel1*delta_t;
    ang2 = ang2+vel2*delta_t;
end

function f= mechanical_energy(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    f=0.5*mass1*(r1^2)*(vel1^2)+0.5*mass2*((r1^2)*(vel1^2)+(r2^2)*(vel2^2)+2*r1*r2*vel1*vel2*cos(ang1-ang2))-(mass1+mass2)*g*r1*cos(ang1)-mass2*g*r2*cos(ang2);
end