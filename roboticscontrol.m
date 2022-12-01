%% controle cinemático
% lei de controle
%addpath rtb common smtb

L(1) = Revolute('d', 0.4, 'alpha', pi/2, 'offset', pi/2);
L(2) = Revolute('a', 0.4318);
L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
L(4) = Revolute('d', 0.4318, 'alpha', pi/2);
L(5) = Revolute('alpha', -pi/2);
L(6) = Revolute();

p560 = SerialLink(L, 'name', 'Puma 560')

q=[0 0 0 0 deg2rad(45) 0]; 
% quinta junta em posicao diferente de 0 para evitar singularidade

%limites de vel das juntas
qdot_lim = pi*[25/18 25/18 25/18 16/9 16/9 7/3];%velocidades limite

%pose desejada
xd = 0.4
yd=-0.65
zd=0.1

pd=[xd yd zd]
Rd=SO3.eul(0, -50, 0) % orientacao desejada
Rd= Rd.R; % transforma o obj SO3 em matriz do matlab
Td = SE3(Rd, pd) % transforma homogenea desejada
%rpyd = rotm2eul(Rd); %transforma a matriz de rotacao em angulos rpy -- default ZYX
rpyd = rotm2axang2(Rd);
lambda =.5
%criacao do plot da simulacao
figure(1)
figure('units','normalized','outerposition', [0 0 1 1])
set(gcf,'Visible','on')
hold on
Td.plot('rgb')
plot_sphere(pd, 0.05, 'y');
p560.plot3d(q)

epsilon = 2e-2; %limiar doerro

% inicializa as variaveis
e=inf(6,1) ; %inicializa erro
t0 = tic; % inicializa clock matlab

while (norm(e) > epsilon)
    T = p560.fkine(q); %transformada homog da pos atual do robo
    J = p560.jacob0(q, 'rpy') %jacob analitica em termos de angulos XYZ rollpityaw

    p = transl(T) % posicao atual do efetuador
    p_til = pd-p; % erro posicao

    %%% calculo da orientacao do robo
    R=SO3(T);
    R=R.R;
    rpy=rotm2eul(R);

    % guarda o 1o vetor de orientacao - evita plot chaveando entre -180
    % e 180
    if i==0
        rpy1st=rpy;
    end

    % calculo erro orientacao
    %evita chaveamento devido à descontinuidade da topologia
    for k=1:3
        if(rpyd(k)-rpy(k)>=-pi & rpyd(k)-rpy(k)<=pi)
            rpy_til(k) = rpyd(k)-rpy(k);
        elseif (rpyd(k)-rpy(k) < -pi)
            rpy_til(k) = rpyd(k)-rpy(k)+2*pi;
        else %if (rpyd(k)-rpy(k)>pi)
            rpy_til(k) = rpyd(k)-rpy(k)+2*pi;
        end
    end

    
    e=[p_til';rpy_til'];%vetor erro
    u=inv(J)*lambda*e;%controlador
    
    tf=toc(t0);%finalizacao do clock do matlab
    t0=tic;%inicializa clock do matlab
    
    % saturacao de velocidade
    for k=1:6
        if u(k)>qdot_lim(k)
            u(k) = qdot_lim(k);
            fprintf("saturou a vel superior da junta", k)
        elseif u(k) < -qdot_lim(k)
            u(k) = -qdot_lim(k);
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

    % para valores nao ficarem chaveando entre 180 e -180
    for k =1:3
        if rpy(k) == -rpy1st(k)
            rpy(k) = rpy1st(k);
        end
    end

    time(i+1)=i %qtd iteracoes

    q=q+qi(end,:)%atualizacao da config do robo
    p560.animate(q)
    i=i+1;

end

fprintf('atingiu objetivo')
hold off


% plot sinal de controle, deslocamento juntas, componentes do erro, norma
% do erro


figure(2)
set(gcf,'Visible','on')
subplot(2,3,1)
hold on
grid on
plot(time, qdotmax(1,:),'r')
plot(time, qdotmin(1,:),'g')
plot(time,control_sig(1,:),'b')
hold off
xlabel('iteracoes')
ylabel('qdot_1(graus/s)')
legend('qdot_(1max)','qdot_(1min)','u_1','Location','Best');
axis([0 length(time)-1 -1.1*(180/pi)*qdot_lim(1) 1.1*(180/pi)*qdot_lim(1)])

subplot(2,3,2)
hold on
grid on
plot(time, qdotmax(2,:),'r')
plot(time, qdotmin(2,:),'g')
plot(time,control_sig(2,:),'b')
hold off
xlabel('iteracoes')
ylabel('qdot_2(graus/s)')
legend('qdot_(2max)','qdot_(2min)','u_2','Location','Best');
axis([0 length(time)-1 -1.1*(180/pi)*qdot_lim(1) 1.1*(180/pi)*qdot_lim(1)])


figure(3)
subplot(2,2,1)
hold on
grid on
plot3(traj(1,:),traj(2,:),traj(3,:))
hold off
xlabel('eixo x')
ylabel('eixo y')
zlabel('eixo z')
legend('caminho percorrido (m)','location','best')
view([60 30])