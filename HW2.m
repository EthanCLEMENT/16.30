s = tf('s');
G = (60*((s/5)+1))/(s*((s/0.4)+1)*(s+1));
G2 = (0.1*(s+0.1))/((s+5)*(s^2+4));
G3 = (30*(s +1))/((s + 0.1)*(s - 2)*(s + 8));
G4 = 1/(s^2 + 3*s + 2);
G5 = 1/((s+1)*(s+2));
Gp = (10*s+1)/((s+1)*((s/3.16)+1)^2);
Gf = 1/((s/10)+1);
Gol = Gp*Gf;

disp("Poles : ");
disp(pole(Gol));

disp("Zeros : ");
disp(zero(Gol));

figure
rlocus(G4);

figure
bode(Gol);

figure 
nyquist(G4);