clear all
close all
quad0 = load('../Log/quad0_global_extr.txt');
quad0_pos = load('../Log/quad0_mat_out.txt');
quad1_pos = load('../Log/quad1_mat_out.txt');

gt_trans_1_to_0 = [6;0;0];

figure(1)
pos_0_x = quad0_pos(:,4);
pos_0_y = quad0_pos(:,5);
pos_0_z = quad0_pos(:,6);
subplot(4,1,1)
plot(pos_0_x,'linewidth',1.2);
hold on
grid on
plot(pos_0_y,'linewidth',1.2);
plot(pos_0_z,'linewidth',1.2);
title('Position of quad_0');
legend('X','Y','Z');
xlabel('time');ylabel('m')

pos_1_x = quad1_pos(:,4);
pos_1_y = quad1_pos(:,5);
pos_1_z = quad1_pos(:,6);
subplot(4,1,2)
plot(pos_1_x,'linewidth',1.2);
hold on
grid on
plot(pos_1_y,'linewidth',1.2);
plot(pos_1_z,'linewidth',1.2);
title('Position of quad_1');
legend('X','Y','Z');
xlabel('time');ylabel('m')

subplot(4,1,3)
G_extrinRot_1_0_x = quad0(:,7);
G_extrinRot_1_0_y = quad0(:,8);
G_extrinRot_1_0_z = quad0(:,9);
plot(G_extrinRot_1_0_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinRot_1_0_y,'linewidth',1.2);
plot(G_extrinRot_1_0_z,'linewidth',1.2);
title('Error of {^{G_0}} R_{G_1}');
legend('Roll','Pitch','Yaw');
xlabel('frame');ylabel('degree')
 
subplot(4,1,4)
G_extrinTrans_1_0_x = quad0(:,10) - gt_trans_1_to_0(1);
G_extrinTrans_1_0_y = quad0(:,11) - gt_trans_1_to_0(2);
G_extrinTrans_1_0_z = quad0(:,12) - gt_trans_1_to_0(3);
plot(G_extrinTrans_1_0_x,'linewidth',1.2);
hold on
grid on
plot(G_extrinTrans_1_0_y,'linewidth',1.2);
plot(G_extrinTrans_1_0_z,'linewidth',1.2);
title('Error of {^{G_0}} p_{G_1}');
legend('X','Y','Z');
xlabel('frame');ylabel('m')