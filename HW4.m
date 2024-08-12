s = tf('s');
K = 100;
Gs = tf([1 K],[1 6 5]);
[A,B,C,D] = tf2ss([1 K],[1 6 5]);

Mo = obsv(A,C);
disp("Observability matrix Mo : ");
disp(Mo);

disp("Rank of Mo : ")
disp(rank(Mo));

Mc = ctrb(A,B);
disp("Controlability matrix Mc : ");
disp(Mc);

disp("Rank of Mc : ");
disp(rank(Mc));

G2s = tf([1 -4 3],[1 6 33 150 200]);

[A1,B1,C1,D1] = tf2ss([1 -4 3],[1 6 33 150 200]);

disp(A1);
disp(B1);
disp(C1);
disp(D1);

[V, DiagA] = eig(A1);
disp("Eigenvector matrix : ");
disp(V);
disp("Diaganolized matrix : ");
disp(DiagA);
disp("Inverted eigenvector Matrix : ");
disp(inv(V));

disp("Az + V^-1Bu : ");
Az = DiagA;
Bz = inv(V)*B1;
disp(Az);
disp(Bz);

disp("Cz + Dzu : " );
Cz = C1*-15*V;
disp(Cz);

A3 = [-2 0 0 0 0 0;
      0 0 1 0 0 0;
      0 -8 -5 0 0 0;
      0 0 0 0 1 0;
      0 0 0 -10 -7 0;
      0 0 0 0 0 -3];
  
B3 = [1 0 0 0;
     0 0 0 0;
     0 1 0 0;
     0 0 0 0;
     0 0 1 0;
     0 0 0 1];
 
C3 = [7 8 2 15 3 5];

D3 = [0 0 0 0];

sys = ss(A3, B3, C3, D3);

disp('State-Space System:');
disp(sys);
sysr = minreal(sys);

disp(sysr);



