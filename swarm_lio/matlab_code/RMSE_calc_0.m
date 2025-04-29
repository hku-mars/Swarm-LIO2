clc
clear all
close all
 
position_offset = [-4;3;2];
yaw_offset = 0; %degree

pose = load('/home/fangcheng/Workspace/swarm_lio_ws/src/multi_uav_lio/swarm_lio/Log/quad0_pose.txt');

[N,cols] = size(pose)
pose_real = zeros(N, cols);
%减去offset
pose_real(:,1) = pose(:,4) - position_offset(1);
pose_real(:,2) = pose(:,5) - position_offset(2);
pose_real(:,3) = pose(:,6) - position_offset(3);

sum_pos = 0;
for i=1:N
    pos_err = [pose(i,1) - pose_real(i,1); pose(i,2) - pose_real(i,2);pose(i,3) - pose_real(i,3)];
    sum_pos = sum_pos + norm(pos_err).^2;
end
RMSE_pos = sqrt(sum_pos/N)
