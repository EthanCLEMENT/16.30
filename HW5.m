disp("Part A : ");

A = [-1 3; 1 6];
B = [1 1; 0 0];
C = [2 -1];
D = [1 0];

Mc = ctrb(A,B);
disp(Mc);
disp(rank(Mc));

Mo = obsv(A,C);
disp(Mo);
disp(rank(Mo));

disp("Part B : ");

A1 = [1 2 1; 1 -3 0; -1 0 -3];
B1 = [0;1;1];
C1 = [1 0 0];
D1 = -1;

Mc1 = ctrb(A1,B1);
disp(Mc1);
disp(rank(Mc1));

Mo1 = obsv(A1,C1);
disp(Mo1);
disp(rank(Mo1));

[V, DiagA] = eig(A1);

disp(C1*V);

disp(rank([DiagA-A1;C1]));

disp("Part C : ");
A2 = [-5 1; 14 0];
B2 = [1;1];
C2 = [1 0];A
D2 = [0];

sys2 = ss(A2,B2,C2,D2);
disp(pole(sys2));
disp(zero(sys2));

[b,a] = ss2tf(A2,B2,C2,D2);
Gs = tf(b,a);

Mc3 = ctrb(A2,B2);
disp(Mc3);
disp(rank(Mc3));

INVMC3 = inv(Mc3);

CHAR = A2^2+13*A2+36*eye(2);

K = [0 1]*INVMC3*CHAR;
disp("K : ");
disp(K);

K2 = place(A2,B2,[-9 -4]);
disp("K2 : ");
disp(K2);

sys2_cl = ss(A2-B2*K, B2,C2,D2);

disp("Part D : ");

M = 8;
P = 1;
A3 = [0 1; 0 0];
B3 = [0; 1/M];
Q = [1 0; 0 0];
R = P^2;

[Ptest, ~, ~] = care(A3, B3, Q, R);

disp("Ptest : ");
disp(Ptest);

Ktest = R^-1 * B3' * Ptest;

disp(Ktest);

K3 = lqr(A3,B3,Q,R);
disp(K3);

sys_cl = ss(A3-K*B3,B3,[1 0],0);


syms P11 P12 P21 P22
RHO = [P11 P12; P21 P22];
RIC = A3'*RHO + RHO*A3 - RHO*B3*inv(R)*B3'*RHO + Q;
disp('Riccati equation:');
disp(RIC);

[V3,diag3] = eig(A3);


% Define the system matrices
A = [0 1; 0 0];
B = [0; 1/8];
C = [1 0];

% Define the time span
tspan = [0 5]; % From t = 0 to t = 5 seconds

% Define the initial conditions
x0 = [0; 10]; % x(0) = [0; 10]

% Define the input function F(t)
F = @(t) 0; % Input is zero

% Define the ODE system
sys = @(t, x) A*x + B*F(t);

% Solve the ODE using ode45
[t, x] = ode45(sys, tspan, x0);

% Extract displacement and velocity
displacement = x(:,1);
velocity = x(:,2);

% Display results at t = 5 seconds
disp(['Displacement at t = 5 s: ', num2str(displacement(end))]);
disp(['Velocity at t = 5 s: ', num2str(velocity(end))]);

% Plot the results
figure;
plot(t, displacement, 'b', 'DisplayName', 'Displacement');
hold on;
plot(t, velocity, 'r', 'DisplayName', 'Velocity');
xlabel('Time (s)');
ylabel('Value');
title('Displacement and Velocity vs. Time');
legend;
grid on;
