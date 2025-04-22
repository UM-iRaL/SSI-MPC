clc; close all; clear;

lw = 1.5;
fs = 6;
ms = 8;
color_ref = 'k';
color_mpc = [0.4660 0.6740 0.1880];
color_gpmpc = [0.6350 0.0780 0.1840]; 
color_olmpc = [0 0.4470 0.7410]; 
color_olmpc_indi = [0.4940 0.1840 0.5560];

%% circle
load("MPC_circle_10.0.mat");
x_mpc = x;

load("OLMPC_circle_10.0_50.mat");
x_olmpc = x;

load("OLMPC_INDI_circle_10.0.mat");
x_olmpc_indi = x;

load("GPMPC_circle_10.0.mat");
x_gpmpc = x;

clear x u w_control

figure(1)
% subplot(6,1,1)
% plot(ref_x(:,1), ref_x(:,2), 'LineWidth', lw); hold on;
% xlabel('$x$ ($m$)','interpreter','latex');
% ylabel('$y$ ($m$)','interpreter','latex');
% legend('Reference Trajectory', 'interpreter', 'latex', 'Location', 'northeast', 'FontSize', fs);

time_all = ref_time(end);
subplot(5,1,1)
plot(ref_time, ref_x(:,1), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,2), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,3), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylabel('Position \ ($m$)','interpreter','latex');
legend('$x$', '$y$', '$z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,2)
plot(ref_time, ref_x(:,4), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,5), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,6), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,7), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylim([-1.2 1.2]);
ylabel('Quaternion','interpreter','latex');
legend('$q_w$', '$q_x$', '$q_y$', '$q_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,3)
plot(ref_time, ref_x(:,8), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,9), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,10), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylim([-12 12]);
ylabel('Velocity \ ($m/s$)','interpreter','latex');
legend('$v_x$', '$v_y$', '$v_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,4)
plot(ref_time, ref_x(:,11), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,12), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,13), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylabel('Angular Velocity \ ($rad/s$)','interpreter','latex')
legend('$\omega_x$', '$\omega_y$', '$\omega_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,5)
plot(ref_time, ref_u(:,1), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,2), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,3), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,4), 'LineWidth', lw); hold on;
xlim([0 time_all]);
xlabel('Time \ ($s$)','interpreter','latex')
ylabel('Normalized Control','interpreter','latex');
legend('$u_1$', '$u_2$', '$u_3$', '$u_4$', 'interpreter', 'latex', 'Location', 'northeast');


figure(2)
t_start = 2001; t_end = 2501;
plot(ref_x(t_start:t_end,1), ref_x(t_start:t_end,2), 'Color', color_ref, 'LineWidth', lw); hold on;
plot(x_mpc(t_start:t_end,1), x_mpc(t_start:t_end,2), 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(x_gpmpc(t_start:t_end,1), x_gpmpc(t_start:t_end,2), 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(x_olmpc(t_start:t_end,1), x_olmpc(t_start:t_end,2), 'Color', color_olmpc, 'LineWidth', lw); hold on;
% plot(x_olmpc_indi(t_start:t_end,1), x_olmpc_indi(t_start:t_end,2), 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
% plot(x_mpc(t_start,1), x_mpc(t_start,2), 's', 'Color', color_mpc, 'MarkerSize', ms); hold on;
% plot(x_mpc(t_end,1), x_mpc(t_end,2), 'o', 'Color', color_mpc, 'MarkerSize', ms); hold on;
% plot(x_gpmpc(t_start,1), x_gpmpc(t_start,2), 's', 'Color', color_gpmpc, 'MarkerSize', ms); hold on;
% plot(x_gpmpc(t_end,1), x_gpmpc(t_end,2), 'o', 'Color', color_gpmpc, 'MarkerSize', ms); hold on;
% plot(x_olmpc(t_start,1), x_olmpc(t_start,2), 's', 'Color', color_olmpc, 'MarkerSize', ms); hold on;
% plot(x_olmpc(t_end,1), x_olmpc(t_end,2), 'o', 'Color', color_olmpc, 'MarkerSize', ms); hold on;
xlabel('$x$ ($m$)','interpreter','latex');
ylabel('$y$ ($m$)','interpreter','latex');
legend('Reference', 'Nominal MPC', 'GP-MPC', 'Ours', 'Location', 'northeast');
% legend('Reference', 'Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');

figure(3)
err_mpc = vecnorm(x_mpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_mpc)
err_gpmpc = vecnorm(x_gpmpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_gpmpc)
err_olmpc = vecnorm(x_olmpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_olmpc)
err_olmpc_indi = vecnorm(x_olmpc_indi(:,1:3)-ref_x(:,1:3),2,2); sum(err_olmpc_indi)
plot(ref_time, cumsum(err_mpc), 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_gpmpc), 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_olmpc), 'Color', color_olmpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_olmpc_indi), 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([0 time_all]);
xlabel('Time ($s$)','interpreter','latex');
ylabel('Tracking Error ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');
clear err_mpc err_gpmpc err_olmpc err_olmpc_indi

%% lemniscate
load("MPC_lemniscate_10.0.mat");
x_mpc = x;

load("OLMPC_lemniscate_10.0_50.mat");
x_olmpc = x;

load("OLMPC_INDI_lemniscate_10.0.mat");
x_olmpc_indi = x;

load("GPMPC_lemniscate_10.0.mat");
x_gpmpc = x;

clear x u w_control

figure(4)
% subplot(6,1,1)
% plot(ref_x(:,1), ref_x(:,2), 'LineWidth', lw); hold on;
% xlabel('$x$ ($m$)','interpreter','latex');
% ylabel('$y$ ($m$)','interpreter','latex');
% legend('Reference Trajectory', 'interpreter', 'latex', 'Location', 'northeast', 'FontSize', fs);

time_all = ref_time(end);
subplot(5,1,1)
plot(ref_time, ref_x(:,1), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,2), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,3), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylabel('Position \ ($m$)','interpreter','latex');
legend('$x$', '$y$', '$z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,2)
plot(ref_time, ref_x(:,4), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,5), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,6), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,7), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylim([-0.5 1.5]);
ylabel('Quaternion','interpreter','latex');
legend('$q_w$', '$q_x$', '$q_y$', '$q_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,3)
plot(ref_time, ref_x(:,8), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,9), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,10), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylim([-12 12]);
ylabel('Velocity \ ($m/s$)','interpreter','latex');
legend('$v_x$', '$v_y$', '$v_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,4)
plot(ref_time, ref_x(:,11), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,12), 'LineWidth', lw); hold on;
plot(ref_time, ref_x(:,13), 'LineWidth', lw); hold on;
xlim([0 time_all]);
ylabel('Angular Velocity \ ($rad/s$)','interpreter','latex')
legend('$\omega_x$', '$\omega_y$', '$\omega_z$', 'interpreter', 'latex', 'Location', 'northeast');

subplot(5,1,5)
plot(ref_time, ref_u(:,1), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,2), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,3), 'LineWidth', lw); hold on;
plot(ref_time, ref_u(:,4), 'LineWidth', lw); hold on;
xlim([0 time_all]);
xlabel('Time \ ($s$)','interpreter','latex')
ylabel('Normalized Control','interpreter','latex');
legend('$u_1$', '$u_2$', '$u_3$', '$u_4$', 'interpreter', 'latex', 'Location', 'northeast');


figure(5)
t_start = 2001; t_end = 2501;
plot(ref_x(t_start:t_end,1), ref_x(t_start:t_end,2), 'Color', color_ref, 'LineWidth', lw); hold on;
plot(x_mpc(t_start:t_end,1), x_mpc(t_start:t_end,2), 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(x_gpmpc(t_start:t_end,1), x_gpmpc(t_start:t_end,2), 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(x_olmpc(t_start:t_end,1), x_olmpc(t_start:t_end,2), 'Color', color_olmpc, 'LineWidth', lw); hold on;
% plot(x_olmpc_indi(t_start:t_end,1), x_olmpc_indi(t_start:t_end,2), 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
% plot(x_mpc(t_start,1), x_mpc(t_start,2), 's', 'Color', color_mpc, 'MarkerSize', ms); hold on;
% plot(x_mpc(t_end,1), x_mpc(t_end,2), 'o', 'Color', color_mpc, 'MarkerSize', ms); hold on;
% plot(x_gpmpc(t_start,1), x_gpmpc(t_start,2), 's', 'Color', color_gpmpc, 'MarkerSize', ms); hold on;
% plot(x_gpmpc(t_end,1), x_gpmpc(t_end,2), 'o', 'Color', color_gpmpc, 'MarkerSize', ms); hold on;
% plot(x_olmpc(t_start,1), x_olmpc(t_start,2), 's', 'Color', color_olmpc, 'MarkerSize', ms); hold on;
% plot(x_olmpc(t_end,1), x_olmpc(t_end,2), 'o', 'Color', color_olmpc, 'MarkerSize', ms); hold on;
xlabel('$x$ ($m$)','interpreter','latex');
ylabel('$y$ ($m$)','interpreter','latex');
legend('Reference', 'Nominal MPC', 'GP-MPC', 'Ours', 'Location', 'northeast');
% legend('Reference', 'Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');


figure(6)
err_mpc = vecnorm(x_mpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_mpc)
err_gpmpc = vecnorm(x_gpmpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_gpmpc)
err_olmpc = vecnorm(x_olmpc(:,1:3)-ref_x(:,1:3),2,2); sum(err_olmpc)
err_olmpc_indi = vecnorm(x_olmpc_indi(:,1:3)-ref_x(:,1:3),2,2); sum(err_olmpc_indi)
plot(ref_time, cumsum(err_mpc), 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_gpmpc), 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_olmpc), 'Color', color_olmpc, 'LineWidth', lw); hold on;
plot(ref_time, cumsum(err_olmpc_indi), 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([0 time_all]);
xlabel('Time ($s$)','interpreter','latex');
ylabel('Tracking Error ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');
clear err_mpc err_gpmpc err_olmpc


%%
GP_data = [316 316 352 334 361 433 343 352 343 406];
sum(GP_data)
