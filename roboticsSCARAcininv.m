%SCARA prova cin inversa
a1=2
a2=1
d4=1

%DH SCARA - Spong pg 84

% 1; a1; 0   ; 0 ; alpha1*
% 2; a2; 180 ; 0 ; alpha2*
% 3; 0 ; 0   ; d*; 0
% 4; 0 ; 0   ; 0 ; alpha4*

% dada a posição [1, 2.73, -3] e a orientação de 
% 30 graus ao redor de x do efetuador com respeito ao 
% sistema de coordenada F0

A1 = Revolute('d', 0,'a',a1,'alpha',deg2rad(0))
A2up = Revolute('d', 0,'a',a2,'alpha',deg2rad(180),'qlim', [0 deg2rad(180)])
A2down = Revolute('d', 0,'a',a2,'alpha',deg2rad(180),'qlim', [deg2rad(-180) 0])

A3 = Prismatic('a',0,'alpha',0,'theta',0,'qlim',[0 10])
A4=Revolute('d', d4,'a',0,'alpha', deg2rad(0))

SCARAup = SerialLink([A1 A2up A3 A4], 'name', 'SCARA')
SCARAdown = SerialLink([A1 A2down A3 A4], 'name', 'SCARA')

% giro de 180 graus em x pois a orientacao de z do efetuador é para baixo
Teff = SE3(SO3.Rz(30).R, [1 2.73 -3])*SE3.Rx(180)

q_calc1 = SCARAup.ikcon(Teff)
q_calc2 = SCARAdown.ikcon(Teff)

q1_calc1 = rad2deg(q_calc1(1))
q2_calc1 = rad2deg(q_calc1(2))
q3_calc1 = q_calc1(3)
q4_calc1 = rad2deg(q_calc1(4))


q1_calc2 = rad2deg(q_calc2(1))
q2_calc2 = rad2deg(q_calc2(2))
q3_calc2 = q_calc2(3)
q4_calc2 = rad2deg(q_calc2(4))

figure(1)
set(gcf, 'Visible', 'on')
view([60 30])
SCARAup.plot(q_calc1);
hold on
Teff.plot('rgb')
hold off


figure(1)
set(gcf, 'Visible', 'on')
view([60 30])
SCARAdown.plot(q_calc2);
hold on
Teff.plot('rgb')
hold off