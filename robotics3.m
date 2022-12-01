% exercicio para determinar o produto de matrizes que resultam na matriz de
% rotacao equivalente

syms phi theta psi

Rphi = SO3.RX(phi)
Rtheta = SO3.Rz(theta)
Rpsi = SO3.Ry(psi)

R = Rpsi*Rphi*Rtheta