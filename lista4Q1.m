clear all
close all
clc
%

%% JACOBIANA PLANAR
% NAO TEM VELOCIDADE Z
% É LINEAR. NAO SAI DO PLANO
% segunda linha da jacobiana é 0
% nao tem velocidade angular nem em x e nem em y
syms theta1 theta2 theta3 a1 a2 a3

%% cinemática direta
A1 = SE3.Rz(theta1)*SE3(0,0,0)*SE3(a1, 0, 0)*SE3.Rx(0);
A2 = SE3.Rz(theta2)*SE3(0,0,0)*SE3(a2, 0, 0)*SE3.Rx(0);
A3 = SE3.Rz(theta3)*SE3(0,0,0)*SE3(a3, 0, 0)*SE3.Rx(0);

H1 = A1;
H2 = simplify(A1*A2); % inercial pro elo 2
H3 = simplify(A1*A2*A3); % inercial pro elo 3
%simplify - > var simbolica

%extrair as translacoes das transformadas
O_00 = [0;0;0];
O_01 = H1.t;
O_02 = H2.t;
O_03 = H3.t;

Z_00 = [0;0;1];
Z_01 = H1.R*[0;0;1];
Z_02 = H2.R*[0;0;1];

%Se a junta for de revolução
J1 = [skew(Z_00)*(O_03 - O_00); Z_00];
J2 = [skew(Z_01)*(O_03 - O_01); Z_01];
J3 = [skew(Z_02)*(O_03 - O_02); Z_02];
%pegar 1o elemento de um produto vetorial e 
%trasnformar a matriz em uma antissimetrica
%travar 1a linha e trava as outras

% se for prismatica : sem vel rotacao
% J1 =  [Z_00; [0;0;0]];
% J2 = [Z_01; [0;0;0]];
% J3 = [Z_02; [0;0;0]];

Jg = simplify([J1,J2,J3])