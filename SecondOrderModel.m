function [A,B,C,D] = SecondOrderModel(omega_n,psi,Gain,Ts)
% ODE function for computing state-space matrices as functions of parameters
A = [0 1;
    -omega_n^2  -2*psi*omega_n];
B = [0;
    Gain*omega_n^2];
C = [1 0;
     -omega_n^2  -2*psi*omega_n]; 

D = [0;Gain*omega_n^2];

end