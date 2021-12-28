function Xdot = ode_3body_fun(~,X) %, G, m1, m2, m3
Xdot = zeros(6, 1);
G = 1;
m1 = 1;
m2 = 2;
m3 = 3;
% Position Vectors
R12 = [X(7)-X(1) X(8)-X(2) X(9)-X(3)];
R23 = [X(13)-X(7) X(14)-X(8) X(15)-X(9)];
R31 = [X(1)-X(13) X(2)-X(14) X(3)-X(15)];
R21 = -R12;
R32 = -R23;
R13 = -R31;
% Position Scalars
r12 = sqrt(R12(1)^2+R12(2)^2+R12(3)^2);
r23 = sqrt(R23(1)^2+R23(2)^2+R23(3)^2);
r31 = sqrt(R31(1)^2+R31(2)^2+R31(3)^2);
r21 = r12;
r32 = r23;
r13 = r31;
%State Vectors:
%%% Body 1:
Xdot(1,1) = X(4); % v1x
Xdot(2,1) = X(5); % v1y
Xdot(3,1) = X(6); % v1z
Xdot(4,1) = -(G*m2/(r21^3))*R21(1)-(G*m3/(r31^3))*R31(1); % a1x
Xdot(5,1) = -(G*m2/(r21^3))*R21(2)-(G*m3/(r31^3))*R31(2); % a1y
Xdot(6,1) = -(G*m2/(r21^3))*R21(3)-(G*m3/(r31^3))*R31(3); % a1z
%%% Body 2:
Xdot(7,1) = X(10); % v2x
Xdot(8,1) = X(11); % v2y
Xdot(9,1) = X(12); % v2z
Xdot(10,1) = -(G*m1/(r12^3))*R12(1)-(G*m3/(r32^3))*R32(1); % a2x
Xdot(11,1) = -(G*m1/(r12^3))*R12(2)-(G*m3/(r32^3))*R32(2); % a2y
Xdot(12,1) = -(G*m1/(r12^3))*R12(3)-(G*m3/(r32^3))*R32(3); % a2z
%%% Body 3;
Xdot(13,1) = X(16); % v3x
Xdot(14,1) = X(17); % v3y
Xdot(15,1) = X(18); % v3z
Xdot(16,1) = -(G*m1/(r13^3))*R13(1)-(G*m2/(r23^3))*R23(1); % a3x
Xdot(17,1) = -(G*m1/(r13^3))*R13(2)-(G*m2/(r23^3))*R23(2); % a3y
Xdot(18,1) = -(G*m1/(r13^3))*R13(3)-(G*m2/(r23^3))*R23(3); % a3z


