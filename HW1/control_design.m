%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all
init_tanks;
g = 9.82;
Tau = 1/alpha1*sqrt(2*tank_h10/g);
k_tank = 60*beta*Tau;
gamma_tank = alpha1^2/alpha2^2;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uppertank = tf(k_tank,[Tau 1]) % Transfer function for upper tank
lowertank = tf(gamma_tank,[(gamma_tank*Tau) 1]) % Transfer function for upper tank
G = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculate PID parameters

% chi = 0.5;
% zeta = 0.7;
% omega0 = 0.1;
% [K_pid,Ti,Td,N]=polePlacePID(chi,omega0,zeta,Tau,gamma_tank,k_tank);
% F = tf([K_pid*(Ti+Td*Ti*N), K_pid*(Ti*N +1), K_pid*N],[Ti, Ti*N 0]);
% sim("tanks.mdl")
% S1 = stepinfo(tank2_out.signals.values,tank2_out.time,50,40)
%     
% chi = 0.5;
% zeta = 0.7;
% omega0 = 0.2;
% [K_pid,Ti,Td,N]=polePlacePID(chi,omega0,zeta,Tau,gamma_tank,k_tank);
% F = tf([K_pid*(Ti+Td*Ti*N), K_pid*(Ti*N +1),K_pid*N],[Ti, Ti*N 0]);
% sim("tanks.mdl")
% S2 = stepinfo(tank2_out.signals.values,tank2_out.time,50,40)

chi = 0.5;
zeta = 0.8;
omega0 = 0.2;
[K_pid,Ti,Td,N]=polePlacePID(chi,omega0,zeta,Tau,gamma_tank,k_tank);
F = tf([K_pid*(Ti+Td*Ti*N), K_pid*(Ti*N +1),K_pid*N],[Ti, Ti*N 0]);
F_best = F;
sim("tanks.mdl")
S3 = stepinfo(tank2_out.signals.values,tank2_out.time,50,40)


OL = F_best*G;
margin(OL)
grid on

[A_f,B_f,C_f,D_f] = ssdata(F_best);
eig_Af = eig(A_f)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 4; % Sampling time

% Discretize the continous controller, save it in state space form

F_dis = c2d(F_best,Ts,'ZOH')
[A_discretized,B_discretized,C_discretized,D_discretized] = ssdata(F_dis);
eig_Ad = eig(A_discretized)

out = sim("tanks_dis.mdl");
tanksDis_zoh_out = out.tanksDis_zoh_out;
S_zoh = stepinfo(tanksDis_zoh_out.signals.values,tanksDis_zoh_out.time,50,40)
figure
plot(tanksDis_zoh_out.time,tanksDis_zoh_out.signals.values)
hold on
plot(tank2_out.time,tank2_out.signals.values)       
grid on
title("Continuous vs ZOH [Ts= "+Ts+"]")
legend("ZOH","Continuous")
grid on

tanksDis_dis_out = out.tanksDis_dis_out;
figure
plot(tanksDis_dis_out.time,tanksDis_dis_out.signals.values)
hold on
plot(tanksDis_zoh_out.time,tanksDis_zoh_out.signals.values)
title("Discretized vs ZOH [Ts="+Ts+"]")
legend("discrete","ZOH")
grid on

S_dis = stepinfo(tanksDis_dis_out.signals.values,tanksDis_dis_out.time,50,40)

%check performance of the response with the requirements
fprintf("RiseTime %f (6)\n", S_dis.RiseTime)
fprintf("overshoot %f(35)\n",S_dis.Overshoot)
fprintf("settlingTime %f (30)\n",S_dis.SettlingTime - 25)


figure
plot(tanksDis_dis_out.time,tanksDis_dis_out.signals.values)
hold on
plot(tank2_out.time,tank2_out.signals.values)
title("Discretized vs Continuous [Ts="+Ts+"]")
legend("discrete","continuous")
grid on



u_zoh = out.u_zoh;
u_dis = out.u_dis;
figure
stairs(u_dis.time,u_dis.signals.values)
hold on
stairs(u_zoh.time,u_zoh.signals.values)
title("u_{dis} vs u_{ZOH} [Ts="+Ts+"]")
legend("discrete","ZOH")
grid on





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Discretize the continous state space system, save it in state space form
A_lin = [-1/Tau 0; 1/Tau -1/(gamma_tank*Tau)];
B_lin = [k_tank/Tau; 0];
C_lin = [0 1];
G_lin = ss(A_lin,B_lin,C_lin,0);
Ts = 4;
G_lin_dis = c2d(G_lin,Ts);
[Phi,Gamma,C,D] = ssdata(G_lin_dis)

% Observability and reachability
Wc = [Gamma Phi*Gamma]
Wo = [C; C*Phi]
rank(Wc)
rank(Wo)

%Continuous poles to discrete poles
G_cl = feedback(F_best*G,1,-1);
poles_cl_cont = pole(G_cl)
poles_c1_dis = exp(poles_cl_cont*Ts)

% State feedback controller gain
L = acker(Phi, Gamma,poles_c1_dis)
% observer gain
K = 1;
% reference gain
lr = 1;

% augmented system matrices
Aa = 1;
Ba = 1;
