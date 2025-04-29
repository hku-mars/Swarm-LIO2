clc
clear all
close all
 
position_offset = [0;3;1.5];
yaw_offset = 0; %degree

pose_esti = load('/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/swarm_lio/Log/quad1_mat_out.txt');
pose_real = load('/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/swarm_lio/Log/quad1_pose.txt');

[N,cols] = size(pose_esti);
traj_length = pose_esti(N,7)

%减去offset
pose_real(:,3) = pose_real(:,3) - yaw_offset;
pose_real(:,4) = pose_real(:,4) - position_offset(1);
pose_real(:,5) = pose_real(:,5) - position_offset(2);
pose_real(:,6) = pose_real(:,6) - position_offset(3);

sum_rot = 0;
sum_pos = 0;
for i=1:N
    rot_err = [pose_esti(i,1) - pose_real(i,1); pose_esti(i,2) - pose_real(i,2);pose_esti(i,3) - pose_real(i,3)];
    pos_err = [pose_esti(i,4) - pose_real(i,4); pose_esti(i,5) - pose_real(i,5);pose_esti(i,6) - pose_real(i,6)];
    sum_rot = sum_rot + norm(rot_err).^2;
    sum_pos = sum_pos + norm(pos_err).^2;
end
RMSE_rot = sqrt(sum_rot/N)
RMSE_pos = sqrt(sum_pos/N)
