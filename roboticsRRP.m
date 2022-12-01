% camera com posicao pc=[-5 15 20]' com respeito à base do manipulador é
% usada para medir a posição pobj de um obj com respeito ao sist de
% coordenadas da propria camera.

q= [0 0 0 ] % pos inicial
l1=2 % elo 1 2 3
l2=2
l3=2

pobj=[-3 2 1 ]'% pos do ob medida pela camera é 
qmanipulador=[-180 90 2]'

% qual a posicao do obj com respeito ao sistema de coordenadas do
% efetuador?

% transformacao do robo para a camera
H_rc = SE3([-5 0 0])*SE3([0 15 0])*SE3([0,0,20])*SE3.Rz(90)*SE3.Rx(180)

% transformacao da camera para o objeto
H_obj=SE3([-3,0,0])*SE3([0,2,0])*SE3([0,0,1])

% transformacao do robo para o efetuador
% Robô : revolucao+revolucao+prismatico
% DH - Ai = Rz(theta_i)*Tz(d_i)*Tx(a_i)*Rx(alfa_i)
% i ; thetai     ;  di      ; ai ; alfai
% 1 ; theta1+90  ; l1       ; 0  ; 90
% 2 ; theta2+90  ; 0        ; 0  ; 90
% 3 ; 0          ; d3+l2+l3 ; 0  ; 0 

theta1=-180
theta2=90
d3=2;

A1=Revolute('d', l1,'a',0,'alpha',deg2rad(90),'offset',deg2rad(90))
A2=Revolute('d', 0,'a',0,'alpha',deg2rad(90),'offset',deg2rad(90))
A3=Prismatic('a',0,'alpha',0,'theta',0,'qlim',[0 10],'offset',l2+l3)

robotRRP = SerialLink([A1 A2 A3], 'name', 'robotRRP')

% teste
q0=[0 0 0]
Tpose0 = robotRRP.fkine(q0)

figure(1)
set(gcf, 'Visible', 'on')
view([60 30])
robotRRP.plot(q0);

q=[deg2rad(theta1) deg2rad(theta2) d3]
H_re = robotRRP.fkine(q) 
% matriz homogenea do robo ate o efetuador dele 

%frame vermelho ao azul

% transformacao do efetuador para o objeto
H_obj = H_re^-1*H_rc*H_cobj

%%cinematica inversa
q=R.ikine(T) %cin inv por otimização sem limites de junta
q=R.ikcon(T) %cin inv por otimizacao com limites de junta
%T = obj SE03 4x4

