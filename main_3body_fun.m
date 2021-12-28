% Brogan Main 3_Body Problem (Fun Script)
clear;clc;
global G
global m1
global m2
global m3
G = 1;
m1 = 1;
m2 = 2;
m3 = 3;

% Initial Conditions: (Try Playing with the Z component of velocity)
%%% Body 1:
Xo(1) = 0;                  % r1x
Xo(2) = 0;                  % r1y
Xo(3) = 0;                  % r1z
Xo(4) = 0;                  % v1x
Xo(5) = 0;                  % v1y
Xo(6) = 1;                % v1z
%%% Body 2:
Xo(7) = 1;                  % r2x
Xo(8) = 0;                  % r2y
Xo(9) = 0;                  % r2z
Xo(10) = 0;                 % v2x
Xo(11) = sqrt(6);           % v2y
Xo(12) = 0;                 % v2z
%%% Body 3:
Xo(13) = 0.5;               % r3x
Xo(14) = sqrt(3)/2;         % r3y
Xo(15) = 0;                 % r3z
Xo(16) = -(3*sqrt(2))/2;    % v3x
Xo(17) = sqrt(6)/2;         % v3y
Xo(18) = 0;                 % v3z

tspan = 0:0.1:10;           % Time span (s)

options = odeset('RelTol',1.0e-13,'InitialStep',1.0e-12,'AbsTol',1.0e-15);

[t, X] = ode45(@ode_3body_fun, tspan, Xo, options);

R1vec = zeros([3, length(tspan)]);
V1vec = zeros([3, length(tspan)]);
R2vec = zeros([3, length(tspan)]);
V2vec = zeros([3, length(tspan)]);
R3vec = zeros([3, length(tspan)]);
V3vec = zeros([3, length(tspan)]);
Rcmvec = zeros([3, length(tspan)]);
Rcm1 = zeros([3, length(tspan)]);
Rcm2 = zeros([3, length(tspan)]);
Rcm3 = zeros([3, length(tspan)]);
Rconnect = zeros([3, length(tspan)]);
k = 1;


for c = 1:length(tspan)
    R1vec(:,c) = [X(c,1); X(c,2); X(c,3)];
    V1vec(:,c) = [X(c,4); X(c,5); X(c,6)];   
    R2vec(:,c) = [X(c,7); X(c,8); X(c,9)];
    V2vec(:,c) = [X(c,10); X(c,11); X(c,12)];
    R3vec(:,c) = [X(c,13); X(c,14); X(c,15)];
    V3vec(:,c) = [X(c,16); X(c,17); X(c,18)];
    Rcmvec(:,c) = (m1*R1vec(:,c)+m2*R2vec(:,c)+m3*R3vec(:,c))/(m1+m2+m3);
    Rcm1(:,c) = R1vec(:,c)-Rcmvec(:,c);
    Rcm2(:,c) = R2vec(:,c)-Rcmvec(:,c);
    Rcm3(:,c) = R3vec(:,c)-Rcmvec(:,c);
    
    Rconnect(:,k) = Rcm1(:,c);
    Rconnect(:,k+1) = Rcm2(:,c);
    Rconnect(:,k+2) = Rcm3(:,c);
    Rconnect(:,k+3) = Rcm1(:,c);
    k = k+4;
end

%%%%%%%%%

figure('units','normalized','outerposition', [0 0 1 1])
c(1) = 1;
v(1) = 4;
for k = 1:length(tspan)
    clf(1)
    plot3(Rcm1(1,k), Rcm1(2,k), Rcm1(3,k), '*m', 'linewidth', 10), grid, hold on
    plot3(Rcm2(1,k), Rcm2(2,k), Rcm2(3,k), '*r', 'linewidth', 10)
    plot3(Rcm3(1,k), Rcm3(2,k), Rcm3(3,k), '*g', 'linewidth', 10)
    %plot3(Rconnect(1,c(k):v(k)), Rconnect(2,c(k):v(k)), Rconnect(3,c(k):v(k)), 'k')
    c(k+1)=c(k)+4;
    v(k+1)=v(k)+4;
    axis([-2 2 -2 2 -2 2])
    xlabel('X'), ylabel('Y'), title('3-Body Trajectory')
    pause(0.02)
end

%Displaying the Motions (Auxiliary plots)
% figure(1)
% plot(Rcm1(1,:), Rcm1(2,:), '*m', 'linewidth', 10), grid, hold on
% plot(Rcm2(1,:), Rcm2(2,:), '*r', 'linewidth', 10), hold on
% plot(Rcm3(1,:), Rcm3(2,:), '*g', 'linewidth', 10), hold on
% plot(Rconnect(1,1:4), Rconnect(2,1:4), 'k')
% plot(Rconnect(1,5:8), Rconnect(2,5:8), 'm')
% plot(Rconnect(1,9:12), Rconnect(2,9:12), 'b')
% plot(Rconnect(1,13:16), Rconnect(2,13:16), 'r')
% axis equal, xlabel('X'), ylabel('Y'), title('3-Body Trajectory Top-Down View')
% 
% figure(2)
% plot3(Rcm1(1,:), Rcm1(2,:), Rcm1(3,:), 'm', 'linewidth', 2), grid, hold on
% plot3(Rcm2(1,:), Rcm2(2,:), Rcm2(3,:), 'r', 'linewidth', 2), hold on
% plot3(Rcm3(1,:), Rcm3(2,:), Rcm3(3,:), 'g', 'linewidth', 2)
% axis equal, xlabel('X'), ylabel('Y'), title('3-Body Trajectory Relative to Center of Mass')
% 
% figure(3)
% plot3(R1vec(1,:), R1vec(2,:), R1vec(3,:), 'm'),grid, hold on
% plot3(R2vec(1,:), R2vec(2,:), R2vec(3,:), 'r'), hold on
% plot3(R3vec(1,:), R3vec(2,:), R3vec(3,:), 'g')
% axis equal, xlabel('X'), ylabel('Y'), zlabel('Z'), title('3-Body Trajectory Relative to Fixed Point in Space')
% 
figure(4)
c = 1;
v = 4;
plot3(Rcm1(1,:), Rcm1(2,:), Rcm1(3,:), '*m', 'linewidth', 10), grid, hold on
plot3(Rcm2(1,:), Rcm2(2,:), Rcm2(3,:), '*r', 'linewidth', 10), hold on
plot3(Rcm3(1,:), Rcm3(2,:), Rcm3(3,:), '*g', 'linewidth', 10), hold on
for j = 1:length(Rconnect)/4 % Makes a triangle between the 3 planets at each time step
    plot3(Rconnect(1,c:v), Rconnect(2,c:v), Rconnect(3,c:v), 'k')
    c = c+4;
    v = v+4;
end
axis equal, xlabel('X'), ylabel('Y'), title('3-Body Trajectory')

