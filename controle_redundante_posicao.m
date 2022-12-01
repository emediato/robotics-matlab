%% controle de manipuladores redundantes
%% command window
%addpath rtb common smtb

% controle de pose - preciso 3DOF posicao x y z
% controle de orientacao - preciso de 3DOF
% trilho da um setimo grau de liberdade para 6DOF de uma tarefa

% juntas que sobram podem fazer uma tarefa secundaria

close all
clear all
clc;

%% definindo se o robo tem 6 ou 7 DOF
n = 6 ;

% definindo simulador coppelia ou RVCTools
coppelia=false;



if n==6 
    % DH DO IRB120
    L(1) = Revolute('d', .290, 'alpha', -pi/2, 'qlim', 11/12*[-pi pi]);
    L(2) = Revolute('a', .270, 'offset', -pi/2, 'qlim', 11/18*[-pi pi]);
    L(3) = Revolute('a', 0.070, 'alpha', -pi/2, 'qlim', [-(11/18)*pi (7/18)*pi]);
    L(4) = Revolute('d', .302, 'alpha', pi/2, 'qlim', 8/9*[-pi pi]);
    L(5) = Revolute('alpha', -pi/2, 'qlim', 2/3*[-pi pi]);
    L(6) = Revolute('d', .072, 'offset', pi, 'qlim', 20/9*[-pi pi]);
    
    il20 = SerialLink(L, 'name', 'IRB 120') % cria roboseriallink
    il20.tool = transl(0,0,.04) % translaciona o sistema de coordenadas
    if coppelia==false
        figure('position',[280 70 560 420])
        %il20.plot(q)
        view([60 30])
    end
end

% posicao para controle de regulacao
pd = [.300 .080 .300];
%valores calculados unicamente para plotagem da simulacao
Rd = SO3.Ry(180) % rotacao em y de 180
Rd=Rd.R; %objeto SO3 -> matriz matlab
Td = SE3(Rd,pd); %transformada homogenea desejada

% limiar do erro  -- simulacao se encerra quando o erro chega nesse valor
epsilon = 2e-2;

e=inf(3,1); % inicializacao do erro
t0=tic; %inicializacao do clock do matlab
i=0;%variaveldecontagem das interacoes


q=[Rd Rd]
%while(norm(e) > epsilon)
T=il20.fkine(q);%transformada homogenea da config atual do robo
Jc=il20.jacob0(q,'rpy'); %jacob analitica completa
J=Jc(1:3,:); %jacobiana analitica de posicao

p= transl(T); % vetor de translacao retirado da transformada homogenea
p_til=pd-p;  % erro de posicao

%% calculo orientacao do robo -- utilizado p plotar os graficos-resposta
R=SO3(T);
R=R.R;
rpy=rotm2eul(R);

e=[p_til]';

u=pinv(J) * lambda * e; % controlador

tf=toc(t0); % finalizacao do clock do matlab

% saturacao velocidade
for k = 1:6
    if u(k) > qdot_lim(k)
        u(k) = qdot_lim(k);
        fprintf('saturou a velocidade superior da junta %d!', k)
    elseif u(k) < -qdot_lim(k)
        u(k) = -qdot_lim(k);
        fprintf('saturou a velocidade inferior da junta %d!', k)
    end
end

%meu objeto de controle eh a velocidade do robo
%entao eu tenho que integrar essa velocidade para verificar o deslocamento
%integrador de primeira ordem
tf_robo = @(t,qi) [u(1);u(2);u(3);u(4);u(5);u(6)];
[~,qi] = ode45(tf_robo, 0:tf:tf, zeros(1,6));

%verifica os limites de deslocamento das juntas

for k = 1:6
    if q(k) + qi(end,k) < il20.qlim(k,1)
        q(k) = il20.qlim(k,1);
        fprintf('atingiu limite inferior da junta %d!', k)
    elseif q(k) + qi(end,k) > il20.qlim(k,2)
        q(k) =  il20.qlim(k);
        fprintf('atingiu limite superior da junta %d!', k)
    end
end


%calculo orientacao do robo
R = SO3(T)
R = R.R;
rpy = rotm2eul(R);

% calculo do erro de orientacao
% evita chaveamento devido a descontinuidade da topologia
% roll pitch yaw tem descontinuidade em -180
for k =1:3
    if (rpyd(k)-rpy(k) >= -pi&rpyd(k)-rpy(k) <= pi)
        rpy_til(k) = rpyd(k)-rpy(k);
    elseif (rpyd(k)-rpy(k) < -pi)
        rpy_til(k) = rpyd(k)-rpy(k)+2*pi;
    else
        rpy_til(k) = rpyd(k)-rpy(k)-2*pi;    
    end
    
end

syms t % var simbolica tempo
wn = pi/10;

% trajetoria desejada
% pds(t) = [.050*sin(wn*t)+.300 .050*cos(wn*t)+0.080 .300];
pds(t) = [.020*(sin(wn*t)+sin(4*wn*t))+.300 .020*(cos(wn*t))+0.080 .300];

