%% command window
%addpath rtb common smtb

%% translaçao

% transformação homogênea (4x4) 
% que representa uma translação pura de x, y and z.
T = transl(1,2,3)


% translação da transformação homogênea T, 
% sendo p um vetor (3x1).
p = transl(T)


% translação homogênea (4x4) que representa uma 
% translação pura de p=[x,y,z].
T = transl(p)

% transl cria matriz!! 


%translação da transformação homogênea T
[x,y,z] = transl(T)


%% rotação

%r = SO3() 
% objeto SO3 que representa uma rotação nula.

%r = SO3(R) 
% objeto SO3 formado a partir de uma matriz de rotação R (3x3).

r = SO3(T)
% objeto SO3 formado a partir da rotação de uma matriz de transformação 
%homogênea T (4x4).

%r = SO3.Rx(theta)/ SO3.Ry(theta)/ SO3.Rz(theta)
% objeto SO3 que representa uma rotação de 'theta' radianos em torno dos 
%eixos x/y/z.

r0 = SO3()

q = pi/2;

rx=SO3.Rx(q)
figure(1)
set(gcf, 'Visible','on')
view([60 30])
%rx.plot('rgb')
rx.animate('rgb')

ry = SO3.Ry(q)  %rotacao em y de 90 graus
r1=rx*ry
r2=ry*rx
figure(2)
r1.plot('rgb')
r2.plot('rgb')

%T.plot : Mostra o sistema de coordenada (plota eixos x, y, z)
%T.animate : Mostra o sistema de coordenada saindo de I até T.

%% Transformação Homogênea
T = SE3() 
% objeto SE3 que representa um movimento nulo.

% T = SE3(x, y, z)
%objeto SE3 que representa uma translação pura, definida por x, y and z.

% T = SE3(xyz)
% objeto SE3 que representa uma translação pura definida pelo vetor xyz (3x1).

% T = SE3(R)
% objeto SE3 que representa a translação pura definida por R, sendo R um objeto SO3.

% T = SE3(R, xyz)
% objeto SE3 que representa uma rotação definida pela matriz de rotação ortonormal R
%(3x3) e uma translação dada pelo vetor xyz (3x1).
R = [-1 0 0; 0 -1 0; 0 0 1];
xyz = [4 5 6]';
T1 = SE3(R, xyz)
figure(3)
set(gcf, 'Visible','on')
view([60 30])
T1.animate('rgb')

