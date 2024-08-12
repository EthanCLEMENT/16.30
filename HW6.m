close all; clear all;

% Define system matrices
A = [1 5; -2 3];
B = [1; 0];
C = [1 0];
D = [0];

% Define cost matrices
Q = eye(2); % Q = I_2
R = 0.01;

% Basic LQR design
Klqr = lqr(A, B, Q, R);

% Compute N for reference tracking
N_bar = inv(-C * inv(A - B * Klqr) * B);

% Closed-loop system for nominal case
Acl = A - B * Klqr;
Bcl = B * N_bar;
Ccl = C;
Dcl = D;
sys_cl_nom = ss(Acl, Bcl, Ccl, Dcl);

% Simulation and plotting
t = 0:0.1:10;
r = ones(size(t)); % Step input
[y, t, x] = lsim(sys_cl_nom, r, t);

figure;
plot(t, y, 'LineWidth', 2);
xlabel('Time');
ylabel('Output');
title('Step Response of Nominal LQR Controller');
grid on;

% Compute N for modified control law
N = inv(-C * inv(A - B * Klqr) * B);

% Closed-loop system with modified control law
Bcl_modified = B * N;
sys_cl_modified = ss(Acl, Bcl_modified, Ccl, Dcl);

% Simulation and plotting
[y_mod, t_mod, x_mod] = lsim(sys_cl_modified, r, t);

figure;
plot(t_mod, y_mod, 'LineWidth', 2);
xlabel('Time');
ylabel('Output');
title('Step Response of Modified LQR Controller with N');
grid on;

% Augmented state-space matrices
A_aug = [A zeros(2, 1); -C 0];
B_aug = [B; 0];
QI = 200; % Choose QI to achieve similar performance
Q_aug = blkdiag(Q, QI);
R_aug = R;

% LQR design for augmented system
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

% Closed-loop system for LQ servo
Acl_aug = A_aug - B_aug * K_aug;
Bcl_aug = [zeros(2, 1); 1];
Ccl_aug = [C 0];
Dcl_aug = D;
sys_cl_servo = ss(Acl_aug, Bcl_aug, Ccl_aug, Dcl_aug);

% Simulation and plotting
[y_servo, t_servo, x_servo] = lsim(sys_cl_servo, r, t);

figure;
plot(t_servo, y_servo, 'LineWidth', 2);
xlabel('Time');
ylabel('Output');
title('Step Response of LQ Servo Controller');
grid on;

% Perturbed system
Delta = -0.1;
A_pert = A + Delta * eye(2);

% Recalculate systems with perturbed A
Acl_pert = A_pert - B * Klqr;
sys_cl_nom_pert = ss(Acl_pert, B * N_bar, C, D);

Acl_aug_pert = [A_pert zeros(2, 1); -C 0] - B_aug * K_aug;
sys_cl_servo_pert = ss(Acl_aug_pert, [zeros(2, 1); 1], [C 0], D);

% Simulation and plotting for perturbed systems
[y_nom_pert, t_nom_pert] = lsim(sys_cl_nom_pert, r, t);
[y_servo_pert, t_servo_pert] = lsim(sys_cl_servo_pert, r, t);

figure;
plot(t_nom_pert, y_nom_pert, 'LineWidth', 2);
hold on;
plot(t_servo_pert, y_servo_pert, 'LineWidth', 2);
xlabel('Time');
ylabel('Output');
title('Step Response with \Delta = -0.1');
legend('Nominal LQR with Perturbation', 'LQ Servo with Perturbation');
grid on;

s = tf('s');
G = (s+8)/((s + 1)*(s + 10));
K = 2.3/s;
a = 5.89;
b = 0.1;
Gc = (s + a)*(s + b)/(s);
GPID = G*Gc;
GPID = GPID*K;
Gclosed =feedback(GPID,1);
figure;
rlocus(Gclosed);
figure
step(Gclosed);
disp(pole(Gclosed));
disp(zero(Gclosed));



