%% command window
%addpath rtb common smtb


%% cinemática e cinemática diferencial

% robôs de cilindros e palitinhos
% na biblioteca, é possível ver o robô comercial

% link
% construtor geral. 1ª junta! Vc pode chamar o tipo de junta desejada

% Prismatic
% construtor para um junta prismática e um elo, utilizando modelo DH padrão.

% Revolute
% construtor de uma junta de revolução e um elo, utilizando o modelo DH padrão.


%revolucao nao tem theta,comando tem valor simbólico
L0 = Link('revolute', 'd',1.2, 'a', 0.3, 'alpha', pi/2)
%L0= Revolute('d',1.2, 'a', 0.3, 'alpha', pi/2)
%deve-se usar radianos nessa funcao link!!

%construtor de uma junta de revolucao e um elo, utilizando DH padrao
L1 = Revolute('d',0,'a',0,'alpha',pi/2,'offset',0)
%d direcao

%construtor de uma junta de prismatica e um elo
%prismatica nao tem d
L2 = Prismatic('a', 0, 'alpha',0, 'theta', pi,'qlim', [0 10])

%SerialLink :  construtor geral.
%L1+L2 : construtor a partir de objetos do tipo 'Link'.

robot = SerialLink([L0 L1 L2], 'name', 'myrobot')

%robot.plot(q)
%Plot do robô com a configuração dada por 'q'.

%% Animate Puma 560
E(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
E(2) = Revolute('d', 0, 'a', 0.4318, 'alpha', 0);
E(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
E(4) = Revolute('d', 0.4318, 'a', 0, 'alpha', pi/2);
E(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2);
E(6) = Revolute('d', 0, 'a', 0, 'alpha', 0);
%robot.animate(q)
%Atualiza a pose atual do robô, criado anteriormente utilizando 'robot.plot'.
p560 = SerialLink(E, 'name', 'Puma 560');
q = [0 0 0 0 0 0]; %variaveis das juntas
%vetorda configuracao
figure(1)
set(gcf,'Visible','on')
p560.plot(q);
% ou 
% mdl_puma650
%p560
%% ******************* cinemática direta

p560.fkine(q);
%T = robot.fkine(q, options)
% é a pose do efetuador do robô, retornando um objeto SE3 para a configuração
% 'q' (1xN).

%eff_pose.plot('rgb')
p560.plot3d(q);
% mdl_puma : NÃO FUNCIONA *****************

% ******************* cinemática direta
%q = R.ikine(T)
% são as coordenadas das juntas (1xN) correpondentes à pose do efetuador
% dada por T, que é um objeto do tipo SE3 (4x4), e N é o número de juntas do
% robô


%% Matrizes anti-simétricas

S = skew(s)
%'S' é a matriz anti-simétrica formada a partir do vetor 's'.
% s = vex(S)
% o comando 'vex' é o inverso da função 'skew'.
s = [1 2 3]';
S = skew(s)
a = vex(S)
%% Matriz Jacobiana


%j0 = robot.jacob0(q, options)
% 'jacob0' retorna a matriz Jacobiana geométrica (6xN) para um robô na
%configuração q (1xN), sendo N o número de juntas do robô. A Jacobiana está
%em relação às coordenadas do mundo. options = 0

% je = robot.jacobe(q, options)
% 'jacobe' retorna a matriz Jacobiana geométrica (6xN) para um robô na
%configuração q (1xN), sendo N o número de juntas do robô. A Jacobiana está
%em relação ao sistema de coordenadas do efetuador.

%options:
%‘trans’: retorna a sub-matriz Jacobiana de translação;
%‘rot’: retorna a sub-matriz Jacobiana de rotação.

qz = [0 0 0 0 0 0];




%%
%animate nao func caso plot nao tenha sido usada anteriormente
%robot precisa ser um obj tipo seriallink

q1=0 % pos juntas
q2=0 % distancia das juntas
q3=0 % rotacao

q = [deg2rad(q1) q2 deg2rad(q3)]%pos inicial
figure(2)
set(gcf,'Visible', 'on')
robot.plot(q)

q1f=45
q2f=5
q3f=20


for q1=0:1.5:q1f
    q=[deg2rad(q1) q2 deg2rad(q3)];
    robot.animate(q);
end

for q1=0:1.5:q2f
    q=[deg2rad(q1) q2 deg2rad(q3)];
    robot.animate(q);
end
for q1=0:1.5:q3f
    q=[deg2rad(q1) q2 deg2rad(q3)];
    robot.animate(q);
end

