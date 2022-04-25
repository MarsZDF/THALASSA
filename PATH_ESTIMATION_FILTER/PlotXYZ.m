figure
Q = [0, 0, 0];
X0 = [1,0,0];
Y0 = [0,1,0];
Z0 = [0,0,1];
hold on
arrow3(Q, X0, 'r'); text(1.03,0,0, 'X^{V}');
arrow3(Q, Y0, 'g'); text(0,1.03,0, 'Y^{V}');
arrow3(Q, Z0, 'b'); text(0,0,1.03, 'Z^{V}');
xlim([-5 5])
ylim([-5 5])
zlim([-5 5])

Maro = eul2rotm([90, -45, 180], 'XYZ');
hold on
X1 = (Maro*X0')'; 
Y1 = (Maro*Y0')';
Z1 = (Maro*Z0')';
X1g = (X1 +0.03);
Y1g = (Y1 + 0.03);
Z1g = (Z1 + 0.03);
arrow3(Q, X1, 'r'); text(X1g(1), X1g(2), X1g(3), 'X^{C}');
arrow3(Q, Y1, 'g'); text(Y1g(1), Y1g(2), Y1g(3), 'Y^{C}');
arrow3(Q, Z1, 'b'); text(Z1g(1), Z1g(2), Z1g(3), 'Z^{C}');
xlim([-5 5])
ylim([-5 5])
zlim([-5 5])