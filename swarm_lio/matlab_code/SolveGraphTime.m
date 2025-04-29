clear all
close all
quad0 = load('../Log/quad0_solve_graph_time_copy.txt');
quad0_ = load('../Log/quad0_solve_graph_time.txt');

figure(1)
subplot(2,1,1)
grid on
hold on
plot(quad0(:,1),'linewidth',1.5);
xlabel("time");ylabel('factor num');

subplot(2,1,2)
grid on
hold on
plot(quad0(:,2),'linewidth',1.5);
xlabel("time");ylabel('solve time/ms');

figure(2)
subplot(2,1,1)
grid on
hold on
plot(quad0_(:,1),'linewidth',1.5);
xlabel("time");ylabel('factor num');
subplot(2,1,2)
grid on
hold on
plot(quad0_(:,2),'linewidth',1.5);
xlabel("time");ylabel('solve time/ms');