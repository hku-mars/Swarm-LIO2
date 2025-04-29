clear all
close all
quad0 = load('../Log/quad0_global_extr.txt');
quad1 = load('../Log/quad1_global_extr.txt');
quad0_pos = load('../Log/quad0_mat_out.txt');
quad1_pos = load('../Log/quad1_mat_out.txt');

gt_trans_1_0 = [3;3;0];
gt_trans_0_1 = [-3;-3;0];

figure(1)
pos_0_x = quad0_pos(:,4);
pos_0_y = quad0_pos(:,5);
pos_0_z = quad0_pos(:,6);
subplot(3,1,1)
plot(pos_0_x,'linewidth',1.2);
hold on
grid on
plot(pos_0_y,'linewidth',1.2);
plot(pos_0_z,'linewidth',1.2);
title('True position of quad_0');
legend('X','Y','Z');
xlabel('time');ylabel('m')

subplot(3,1,2)
G_extrinRot_1_0_x = quad0(:,7);
G_extrinRot_1_0_y = quad0(:,8);
G_extrinRot_1_0_z = quad0(:,9);
plot(G_extrinRot_1_0_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinRot_1_0_y,'linewidth',1.2);
plot(G_extrinRot_1_0_z,'linewidth',1.2);
title('Error of {^{G_i}} R_{G_j}');
legend('X','Y','Z');
xlabel('frame');ylabel('degree')
 
subplot(3,1,3)
G_extrinTrans_1_0_x = quad0(:,10) - gt_trans_1_0(1);
G_extrinTrans_1_0_y = quad0(:,11) - gt_trans_1_0(2);
G_extrinTrans_1_0_z = quad0(:,12) - gt_trans_1_0(3);
plot(G_extrinTrans_1_0_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinTrans_1_0_y,'linewidth',1.2);
plot(G_extrinTrans_1_0_z,'linewidth',1.2);
title('Error of {^{G_i}} p_{G_j}');
legend('X','Y','Z');
xlabel('frame');ylabel('m')


figure(2)
pos_1_x = quad1_pos(:,4);
pos_1_y = quad1_pos(:,5);
pos_1_z = quad1_pos(:,6);
subplot(3,1,1)
plot(pos_1_x,'linewidth',1.2);
hold on
grid on
plot(pos_1_y,'linewidth',1.2);
plot(pos_1_z,'linewidth',1.2);
title('True position of quad_1');
legend('X','Y','Z');
xlabel('time');ylabel('m')


G_extrinRot_0_1_x = quad1(:,1);
G_extrinRot_0_1_y = quad1(:,2);
G_extrinRot_0_1_z = quad1(:,3);
subplot(3,1,2)
plot(G_extrinRot_0_1_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinRot_0_1_y,'linewidth',1.2);
plot(G_extrinRot_0_1_z,'linewidth',1.2);
title('Error of {^{G_i}} R_{G_j}');
legend('X','Y','Z');
xlabel('frame');ylabel('degree')

G_extrinTrans_0_1_x = quad1(:,4) - gt_trans_0_1(1);
G_extrinTrans_0_1_y = quad1(:,5) - gt_trans_0_1(2);
G_extrinTrans_0_1_z = quad1(:,6) - gt_trans_0_1(3);
subplot(3,1,3)
plot(G_extrinTrans_0_1_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinTrans_0_1_y,'linewidth',1.2);
plot(G_extrinTrans_0_1_z,'linewidth',1.2);
title('Error of {^{G_i}} p_{G_j}');
legend('X','Y','Z');
xlabel('frame');ylabel('m') 


quad0_traj = load('/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/swarm_lio/Log/quad0_traj.txt');
figure(3)
grid on
hold on

plot3(quad0_traj(:,1), quad0_traj(:,2),quad0_traj(:,3),'linewidth',1.5);
plot3(quad0_traj(:,4), quad0_traj(:,5),quad0_traj(:,6),'linewidth',1.5);
plot3(quad0_traj(:,7), quad0_traj(:,8),quad0_traj(:,9),'linewidth',1.5);
xlabel('X');ylabel('Y');zlabel('Z');
legend('Temp tracker traj','uav traj original','uav traj after transformation');


