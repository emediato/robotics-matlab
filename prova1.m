%% prova 1
%addpath rtb common smtb
% considere 2 sistemas de coordenadas: um inercial 
% dado por Fi

% e o de um corpo, dado por Fc.
% inicialmente com posição e orientação iguais

% o corpo sofre a seguinte sequencia de mov rígidos


% CORRENTE = POS
% INERCIAL = PRE 

H = SE3.Ry(180) % rot 180 Yc
H = SE3(2,0,0)*H % trans 2m xi
H = SE3.Rx(-90)*H % rot -90 em xi
H = H*SE3(0,3,0) % trans 3m yc
H = SE3.Ry(-180)*H % rot -180 em yi
H = H*SE3(0,0,1) % trans 1m zc

p =transl(H)' % vetor posicao da MTH

R = H.R % matriz rotacao da MTH
%retorna matriz e nao obj

