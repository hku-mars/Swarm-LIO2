clear all
close all

pose_esti = load('/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/Log/quad0_mat_out.txt');
pose_real = load('/home/fangcheng/workspace/uav_lio_ws/src/swarm_lio/Log/quad0_pose.txt');

offset = [0;0;1];

pose_esti_x = pose_esti(:,4);
pose_esti_y = pose_esti(:,5);
pose_esti_z = pose_esti(:,6);

pose_real_x = pose_real(:,4) - offset(1);
pose_real_y = pose_real(:,5) - offset(2);
pose_real_z = pose_real(:,6) - offset(3);

N = length(pose_esti_x);

sum = 0;
for i=1:N
    err = [pose_esti_x(i) - pose_real_x(i); pose_esti_y(i) - pose_real_y(i);pose_esti_z(i) - pose_real_z(i)];
    sum = sum + norm(err).^2;
end

RMSE = sqrt(sum/N)

