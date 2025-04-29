clc
clear all
close all
% syms x y z real
euler_angle= [0;-90;90]/180*pi;
Rot_Mat = rotationVectorToMatrix(euler_angle)



P = zeros(3,20);
Q = zeros(3,20);
for i = 1:1:20
    x = -3*i + 10;
    y = 8*i - 20;
    z = 10 ;
    P(1,i) = x;
    P(2,i) = y;
    P(3,i) = z;
    Q(:,i) = Rot_Mat * P(:,i) + [5;3;7];
end

figure(1)
plot3(P(1,:),P(2,:),P(3,:));
hold on
grid on
plot3(Q(1,:),Q(2,:),Q(3,:));
xlabel('X');ylabel('Y');zlabel('Z');

P1 = zeros(3,20);
Q1 = zeros(3,20);

MEAN_Q = mean(Q')';
Q1(1,:) = Q(1,:) - MEAN_Q(1);
Q1(2,:) = Q(2,:) - MEAN_Q(2);
Q1(3,:) = Q(3,:) - MEAN_Q(3);

MEAN_P = mean(P')';
P1(1,:) = P(1,:) - MEAN_P(1);
P1(2,:) = P(2,:) - MEAN_P(2);
P1(3,:) = P(3,:) - MEAN_P(3);

figure(2)
plot3(P1(1,:),P1(2,:),P1(3,:));
hold on
grid on
plot3(Q1(1,:),Q1(2,:),Q1(3,:));
xlabel('X');ylabel('Y');zlabel('Z');

H = P1 * Q1';

rank = rank(H);
[U,S,V] = svd(H);
mirror = [1,0,0;0,1,0;0,0,det(V*U')];
Rot = V*mirror*U'

trans = MEAN_Q - Rot * MEAN_P

Q2 = zeros(3,20);
for i = 1:1:20
    Q2(:,i) = Rot * P(:,i) + trans;
end
figure(3)
plot3(Q(1,:),Q(2,:),Q(3,:));
hold on
grid on
plot3(Q2(1,:),Q2(2,:),Q2(3,:));
xlabel('X');ylabel('Y');zlabel('Z');

