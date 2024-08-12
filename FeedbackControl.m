% Define the transfer function
s = tf('s');
G = (s+2) / (s*(s+1)*(s+5));

% Define the lead compensator
zlead = 1;  % zero of the lead compensator
plead = 200.95;  % pole of the lead compensator
zlag = 0.95;
plag = 0.1;
Gc = ((s + zlead)*(s + zlag))/ ((s + plead)*(s + plag));

% Combined system
L = G * Gc;

% Plot root locus
figure;
rlocus(L);
title('Root Locus of the Compensated System');

% Choose a gain K based on the root locus
K = 21025; % Example gain value
T = feedback(K * L, 1);

% Plot step response of the closed-loop system
figure;
step(T);
title('Step Response of the Compensated Closed-Loop System');

figure; 
step(T/s);
title('Ramp Response');

a=[-1 1.5;1 -2];b=[1 0]';c=[1 0];d=0;
%
% estimator gain calc
l=place(a',c',[-3 -4]);
disp(l);