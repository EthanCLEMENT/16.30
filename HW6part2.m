close all; clear all;

% s = tf('s');
% G = (s+8)/((s + 1)*(s + 10));
% K = 6.84/s;
% a = 2.8;
% b = 0.1;
% Gc = ((s + a)*(s + b))/(s);
% 
% G_PD = G*(s+a)*(s+b)/s;
% G_PD = G_PD*K;
% 
% G_PD = feedback(G_PD,1);
% 
% figure
% rlocus(G_PD);
% 
% figure
% step(G_PD);
% 
% disp(pole(G_PD));
% disp(zero(G_PD));
% 
% Kd = K;
% Kp = K*(a+b);
% Ki = K*a*b;
% 
% C = Kp + Ki/s + Kd*s;
% % Closed-loop system
% sys_cl = feedback(C*G, 1);
% 
% rlocus(sys_cl);
% figure
% step(sys_cl);

A = [0 1; -10 -11];
B = [0;1];
C = [8 1];
D = 0;

sys_ss = ss(A,B,C,D);

% Controllability matrix
Mc = ctrb(A,B);
disp('Controllability Matrix:');
disp(Mc);
disp('Rank of Controllability Matrix:');
disp(rank(Mc));

% Desired characteristic polynomial for controller
char = A^2 + 6*A + 13*eye(2);
K = [0 1] * inv(Mc) * char;
disp('State Feedback Gain K:');
disp(K);

% Compute N_bar
N_bar = inv(-C * inv(A - B * K) * B);
disp('N_bar:');
disp(N_bar);

% Closed-loop system
Acl = A - B * K;
Bcl = B * N_bar;
Ccl = C;
Dcl = D;
sys_ss_cl = ss(Acl, Bcl, Ccl, Dcl);

figure;
rlocus(sys_ss_cl);
title('Root Locus of Closed-Loop System');

figure;
step(sys_ss_cl);
title('Step Response of Closed-Loop System');

% Observability matrix
Mo = obsv(A,C);
disp('Observability Matrix:');
disp(Mo);
disp('Rank of Observability Matrix:');
disp(rank(Mo));

% Desired characteristic polynomial for observer
char2 = A^2 + 12*A + 40*eye(2);
L = char2 * inv(Mo) * [0;1];
disp('Observer Gain L:');
disp(L);

% Augmented system for the compensator
A_aug = [A-B*K  B*K;
         zeros(size(A)) A-L*C];
B_aug = [B*N_bar;
         zeros(size(B))];
C_aug = [C  zeros(size(C))];
D_aug = D;

sys_comp = ss(A_aug, B_aug, C_aug, D_aug);

figure;
rlocus(sys_comp);
title('Root Locus of the Compensator');

figure;
step(sys_comp);
title('Step Response of the Compensator');

Ac = A-B*K-L*C;
Bc = L;
Cc = K;
Dc = D;

Gc = ss(Ac, Bc, Cc, Dc);

figure;
step(Gc);
disp("Eig of Gc : ");
disp(eig(Ac));
figure;
rlocus(Gc);

Al=[A B*Cc;zeros(2) Ac];
Bl=[zeros(2,1);Bc];
Cl=[C zeros(1,2)];
Dl=0;

disp("Eig of Al : ");
disp(eig(Al));

figure;
rlocus(Al,Bl,Cl,Dl);

Acl=Al-Bl*Cl;Bcl=Bl;Ccl=Cl;Dcl=D;
N=inv(Ccl*inv(-Acl)*Bcl);

Gcl=ss(Acl,Bcl*N,Ccl,Dcl*N);
figure;
step(Gcl);
disp("Eig of Gcl : ");
disp(eig(Acl));
figure;
rlocus(Gcl);

Bcl=[-B;-B];
N=inv(Ccl*inv(-Acl)*Bcl)
Gnew=ss(Acl,Bcl*N,Ccl,Dcl*N);

figure;
step(Gnew);
disp("Eig of Gnew : ");
disp(eig(Acl));
figure;
rlocus(Gnew);

