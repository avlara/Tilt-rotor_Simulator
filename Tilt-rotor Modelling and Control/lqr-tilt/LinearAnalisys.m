format long

clear all
clc

ArthurModel

aux1 = zeros(16,4);
aux2 = zeros(4,4);


Aint = [A aux1;
        C aux2];
    
Bint = [B;aux2];

Caux = zeros(4);

Cint = [C,Caux];

D = zeros(4,4);

%sys = ss(Aint,Bint,Cint,Dint);
sys = ss(A,B,C,D);

T = 12*(10^(-3));

sysd = c2d(sys,T,'zoh');

[Ad, Bd, Cd, Dd] = ssdata(sysd);

[~,~,~,~,k] = ctrbf(Ad,Bd,Cd);
ctrl = sum(k)
% Controlável

C2 = eye(16);
[~,~,~,~,k] = obsvf(A,B,C);
obsv = sum(k)


% Observável

% 1/max ^2

% Matriz que pondera os sinais de controle
 R = [0.19        0             0           0;
          0          0.19       0           0;
          0             0              250      0;
          0             0             0        250];
 %R = [1/(3.930^2)        0             0           0;
 %         0          1/(3.930^2)       0           0;
 %         0             0          1/(0.300^2)      0;
 %         0             0             0        1/(0.300^2)];

%R = [1/(4.930^2)        0             0           0;
%          0          1/(4.930^2)       0           0;
%          0             0          1/(0.300^2)      0;
%          0             0             0        1/(0.300^2)];

          
          
% Matriz que pondera os estados do sistema
Q(1,1) = 5;% x
Q(2,2) = 5;% y
Q(3,3) = 5;% z
Q(4,4) = 4*pi; % roll
Q(5,5) = 4*pi; % pitch
Q(6,6) = 15*pi; % yaw
Q(7,7) = 40*pi; % aR
Q(8,8) = 40*pi; % aL
Q(9,9) = 1/4; % dx
Q(10,10) = 1/4; % dy
Q(11,11) = 1/4; % dz
Q(12,12) = 1/(pi/3)^2; % droll
Q(13,13) = 1/(pi/3)^2; % dpitch
Q(14,14) = 1/(pi/4)^2; % dyaw
Q(15,15) = 5/(3*pi)^2; % daR
Q(16,16) = 5/(3*pi)^2; % daL
%Q(17,17) = 50; % intX
%Q(18,18) = 50; % intY
%Q(19,19) = 50; % intZ
%Q(20,20) = 1; % intyaw

% Q(1,1) = 1/(100)^2;% x
% Q(2,2) = 1/(100)^2;% y
% Q(3,3) = 1/(100)^2;% z
% Q(4,4) = 1/(pi/2)^2; % roll
% Q(5,5) = 1/(pi/2)^2; % pitch
% Q(6,6) = 1/(pi/2)^2; % yaw
% Q(7,7) = 1/(pi/2)^2; % aR
% Q(8,8) = 1/(pi/2)^2; % aL
% Q(9,9) = 1/(0.5)^2; % dx
% Q(10,10) = 1/(0.5)^2; % dy
% Q(11,11) = 1/(0.5)^2; % dz
% Q(12,12) = 1/(pi/3)^2; % droll
% Q(13,13) = 1/(pi/3)^2; % dpitch
% Q(14,14) = 1/(pi/4)^2; % dyaw
% Q(15,15) = 1/(3*pi)^2; % daR
% Q(16,16) = 1/(3*pi)^2; % daL
% Q(17,17) = 10; % intX
% Q(18,18) = 10; % intY
% Q(19,19) = 9; % intZ
% Q(20,20) = 8; % intyaw


[K,~,~] = dlqr(Ad,Bd,Q,R);

% valores dos sinais de entrada do sistema em equlíbrio
Data;

%AUX = zeros(16,16);
AUX = 0.23*eye(16);

InitialValue = [0;0;0;rolleq;pitcheq;0;aReq;aLeq;0;0;0;0;0;0;0;0;reshape(AUX,256,1)];

