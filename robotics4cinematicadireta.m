%cinematica direta

q1=0
q2=45
q3=30
q4=0
q5=0
q6=0

q = [rad2deg(q1) rad2deg(q2) rad2deg(q3) rad2deg(q4) rad2deg(q5) rad2deg(q6)]

eff_pose = p560.fkine(q)

figure(1)
set(gcf,'Visible','on')
view([60 30])
p560.plot(q)
hold on
eff_pose.plot('rgb')
hold off
