clear all;
close all;

lw = 1.5;
fs = 6;
ms = 8;
color_ref = 'k';
color_mpc = [0.4660 0.6740 0.1880];
color_gpmpc = [0.6350 0.0780 0.1840]; 
color_olmpc = [0 0.4470 0.7410]; 
color_olmpc_indi = [0.4940 0.1840 0.5560];

%%
% Experiment data in the following format:
% [ref_v_max (m/s)  track_err_pos (m)  traj_v_max (m/s)  opt_time (ms)]

% Circle 
max_speed = [2.5683 5.0683 7.5683 10.0683 12.5683];     

mpc = [  
         2.50  0.06426  2.52500  1.321;
         5.00  0.12420  4.88300  1.209;
         7.50  0.17119  7.13800  1.187;
         10.0  0.22035  9.31300  1.150;
         12.5  0.26128  11.4640  1.146;
                                          ];

gpmpc = [ 
         2.50  0.03030  2.54600  2.970;
         5.00  0.04624  4.99300  2.764;
         7.50  0.07177  7.25700  2.659;
         10.0  0.11062  9.58700  2.620;
         12.5  0.16626  0.16626  2.610;
                                          ];

% % n_rf=100
% olmpc = [ 
%          2.50  0.01201  2.5520   6.670;
%          5.00  0.02302  5.0260   6.065;
%          7.50  0.02878  7.4960   5.875;
%          10.0  0.05317  9.9670   5.734;
%          12.5  0.08774  12.276   5.715;
%                                           ];

% n_rf=50
olmpc = [  
         2.50  0.01022  2.5560   4.728;
         5.00  0.01781  5.0400   4.327;
         7.50  0.02979  7.4940   4.135;
         10.0  0.05349  9.9680   4.101;
         12.5  0.09616  12.319   4.067;
                                          ];

olmpc_indi = [  
         2.50  0.00431  2.5660   4.890;
         5.00  0.01127  5.0550   4.493;
         7.50  0.02072  7.5490   4.358;
         10.0  0.02920  10.047   4.242;
         12.5  0.06083  12.312   4.249;
                                          ];

figure(1)
plot(mpc(:,1), mpc(:,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(gpmpc(:,1), gpmpc(:,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(olmpc(:,1), olmpc(:,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
plot(olmpc_indi(:,1), olmpc_indi(:,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.5 12.5]);
xlabel('Maximum Velocity ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');

% Lemniscate
max_speed = [3.4562 7.0937 10.5022 14.1083 17.7283];

mpc = [  
         2.50  0.05936  1.8520   1.294;
         5.00  0.11660  4.6480   1.213;
         7.50  0.15889  7.3650   1.166;
         10.0  0.19577  9.8500   1.146;
%         12.0  0.25291  12.470   1.153;
         12.5  0.29449  12.832   1.145;
                                          ];

gpmpc = [  
         2.50  0.03017  1.8750   2.979;
         5.00  0.04172  4.8170   2.825;
         7.50  0.07212  7.5070   2.678;
         10.0  0.09982  9.9460   2.649;
%          12.0  0.15671  12.507   2.597;
         12.5  0.20051  12.937   2.605;
                                          ]; 

% % n_rf=100
% olmpc = [  
%          2.50  0.01269  1.8790   6.545;
%          5.00  0.02516  4.8170   6.039;
%          7.50  0.04853  7.5300   5.839;
%          10.0  0.07221  9.9580   5.754;
% %          12.0  0.13112  12.831   5.699;
%          12.5  0.17457  12.937   5.661;
%                                           ];

% n_rf=50
olmpc = [  
         2.50  0.01311  1.8790   4.780;
         5.00  0.02531  4.8180   4.298;
         7.50  0.04325  7.4730   4.158;
         10.0  0.07216  9.9580   4.113;
         12.5  0.17629  13.060   4.054;
                                          ];

olmpc_indi = [  
         2.50  0.00591  1.8760   4.955;
         5.00  0.01519  4.8090   4.516;
         7.50  0.03463  7.5070   4.372;
         10.0  0.05554  9.9340   4.207;
         12.5  0.16404  12.821   4.195;
                                          ];

figure(2)
plot(mpc(:,1), mpc(:,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
plot(gpmpc(:,1), gpmpc(:,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
plot(olmpc(:,1), olmpc(:,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
plot(olmpc_indi(:,1), olmpc_indi(:,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.5 12.5]);
xlabel('Maximum Velocity ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northeast');

                                                                 
