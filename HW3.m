y = -0.15;
v = 50.8691;
g = 9.81;
n = 0.9887;
k1 = 61.6594;
k2 = 4.8747e-5;

A = [(sin(y)*g)/v, ((cos(y)*g)/v^2) - ((n*g)/v^2); -cos(y), (2*k1*n^2*g)/v^3 - (-2*k2*v*g)];
B = [g/v^2;(-2*k1*n*g)/v^2];
Cv = [0 1];
Cy = [1 0];
D = 0;

sysy = ss(A,B,Cy,D);
sysv = ss(A,B,Cv,D);

[b,a] = ss2tf(A,B,Cy,D);
[b2,a2] = ss2tf(A,B,Cv,D);

Gy = tf(b,a);
Gv = tf(b2,a2);

Gy_cl = feedback(Gy*-50,1);

disp("Roots of Gy(s) : ");
disp(pole(Gy_cl));

disp("Roots of Gv(s) : ");
disp(pole(Gv));

disp("Zeros of Gy(s) : ");
disp(zero(Gy_cl));

figure
rlocus(Gy_cl);

figure
bode(Gy_cl);



