clear all;
close all;


%%
% Trajectory tracking experiment
% Trajectory: Circle, Lemniscate, wrapped Circle, wrapped Lemniscate, Tilted Circle
% Algorithm: Nominal MPC, GP-MPC, OL-MPC, and OL-MPC (w/ INDI)
% For each trajectory type, five reference maximal axial velocities are specified (giving five maximal speeds) 
% For each reference maximal axial velocity, each algorithm is run five times (row of the recorded results)


% plot setup
lw = 1.5;
fs = 6;
ms = 8;
color_ref = 'k';
color_mpc = [0.4660 0.6740 0.1880];
color_gpmpc = [0.6350 0.0780 0.1840]; 
color_olmpc = [0 0.4470 0.7410]; 
color_olmpc_indi = [0.4940 0.1840 0.5560];


%%
% ref_v_max = [2.50 5.00 7.50 10.0 12.5];

%% Circle 
max_speed_circle = [2.5683 5.0683 7.5683 10.0683 12.5683];     

mpc = [  
         0.06426  0.06384  0.06384  0.06384  0.06384;
         0.12420  0.12388  0.12412  0.12412  0.12412;
         0.17119  0.17489  0.17472  0.17489  0.17489;
         0.22035  0.22037  0.21753  0.21755  0.21743;
         0.26128  0.25925  0.26150  0.26151  0.26160;
                                                        ];

gpmpc = [ 
         0.03030  0.03015  0.03015  0.03015  0.03015;
         0.04624  0.04625  0.04624  0.04629  0.04629;
         0.07177  0.07180  0.07181  0.07177  0.07181;
         0.11062  0.11057  0.11062  0.11061  0.11060;
         0.16626  0.16384  0.16630  0.16631  0.16627;
                                                        ];

% n_rf=50
olmpc = [  
         0.01022  0.01189  0.01177  0.01157  0.01170;
         0.01781  0.02246  0.02267  0.02266  0.02292;
         0.02979  0.03414  0.03434  0.03451  0.03399;
         0.05349  0.05030  0.04990  0.05377  0.05138;
         0.09616  0.09102  0.09483  0.09165  0.09711;
                                                        ];

olmpc_indi = [  
         0.00431  0.00313  0.00317  0.00308  0.00304;
         0.01127  0.00891  0.00820  0.00802  0.00868;
         0.02072  0.01990  0.01885  0.02005  0.01976;
         0.02920  0.02618  0.02596  0.02773  0.02994;
         0.06083  0.05714  0.06254  0.06896  0.05553;
                                                        ];

figure(1)
errorbar(max_speed_circle, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
errorbar(max_speed_circle, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_circle, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_circle, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.0 13.0]);
% title('Tracking Performance with Circle Trajectory')
xlabel('Maximum Speed ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');



%% Lemniscate
max_speed_lemniscate = [3.4562 7.0937 10.5022 14.1083 17.7283];

mpc = [  
         0.05936  0.05896  0.05935  0.05935  0.05935;
         0.11660  0.11668  0.11670  0.11670  0.11671;
         0.15889  0.15857  0.15871  0.15857  0.15871;
         0.19577  0.19552  0.19553  0.19553  0.19553;
         0.29449  0.28904  0.29754  0.29770  0.29754;
                                                        ];

gpmpc = [  
         0.03017  0.03078  0.03078  0.02998  0.02998;
         0.04172  0.04172  0.04183  0.04183  0.04172;
         0.07212  0.06834  0.06827  0.06819  0.06826;
         0.09982  0.09983  0.09985  0.09980  0.09987;
         0.20051  0.20059  0.20022  0.20124  0.19826;
                                                        ]; 

% n_rf=50
olmpc = [  
         0.01311  0.01297  0.01258  0.01282  0.01307;
         0.02531  0.02472  0.02554  0.02519  0.02482;
         0.04325  0.04151  0.04172  0.04392  0.04179;
         0.07216  0.07228  0.07178  0.07074  0.07198;
         0.17629  0.18450  0.17445  0.17493  0.17881;
                                                        ];

olmpc_indi = [  
         0.00591  0.00609  0.00613  0.00626  0.00589;
         0.01519  0.01462  0.01408  0.01324  0.01429;
         0.03463  0.02876  0.02788  0.02813  0.02800;
         0.05554  0.05424  0.05905  0.05689  0.05475;
         0.16404  0.16306  0.16589  0.16335  0.16908;
                                                        ];

figure(2)
errorbar(max_speed_lemniscate, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
errorbar(max_speed_lemniscate, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_lemniscate, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_lemniscate, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.0 18.0]);
% title('Tracking Performance with Lemniscate Trajectory')
xlabel('Maximum Speed ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');
                                                      


%% wrapped Circle 
max_speed_wrapped_circle = [2.8618 5.6244 8.4511 11.2527 14.0502];

mpc = [  
         0.06252  0.06288  0.06287  0.06288  0.06287;
         0.11948  0.11955  0.11955  0.11955  0.11954;
         0.17026  0.17023  0.17023  0.17023  0.17024;
         0.21562  0.21588  0.21591  0.21589  0.21590;
         0.33123  0.33111  0.33101  0.33105  0.33109;
                                                        ];

gpmpc = [  
         0.02950  0.02951  0.02950  0.02949  0.02949;
         0.04600  0.04575  0.04574  0.04573  0.04574;
         0.07359  0.07373  0.07373  0.07371  0.07371;
         0.11927  0.11932  0.11922  0.11919  0.11922;
         0.24744  0.24706  0.24748  0.24746  0.24799;
                                                        ];

% n_rf=50
olmpc = [  
         0.01008  0.01002  0.01020  0.00985  0.00990;
         0.01962  0.01966  0.01960  0.01966  0.01966;
         0.03806  0.03776  0.03771  0.03745  0.03751;
         0.06911  0.06550  0.06530  0.06863  0.06472;
         0.24176  0.24143  0.24433  0.24290  0.24489;
                                                        ];

olmpc_indi = [  
         0.00374  0.00370  0.00348  0.00348  0.00354;
         0.01162  0.01140  0.01159  0.01117  0.01233;
         0.02359  0.02453  0.02373  0.02344  0.02299;
         0.03997  0.03875  0.04076  0.04121  0.04461;
         0.21566  0.21271  0.21877  0.21084  0.21075;
                                                        ];


figure(3)
errorbar(max_speed_wrapped_circle, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_circle, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_circle, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_circle, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.0 15.0]);
% title('Tracking Performance with wrapped Circle Trajectory')
xlabel('Maximum Speed ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');  


%%
% ref_v_max = [2.00 4.00 6.00 8.00 10.0];

%% wrapped Lemniscate (change in height: 3m)
max_speed_wrapped_lemniscate = [3.0403 5.9711 8.9027 11.8201 14.7203];


mpc = [  
         0.05122  0.05172  0.05172  0.05169  0.05169;
         0.10037  0.10036  0.10037  0.10037  0.10037;
         0.14571  0.14572  0.14571  0.14572  0.14569;
         0.19311  0.19307  0.19296  0.19307  0.19301;
         0.29934  0.29935  0.29917  0.29960  0.29931  ;
                                                        ];

gpmpc = [  
         0.02870  0.02893  0.02893  0.02893  0.02893;
         0.04530  0.04529  0.04529  0.04529  0.04529;
         0.06373  0.06371  0.06376  0.06375  0.06372;
         0.10597  0.10573  0.10587  0.10571  0.10599;
         0.20509  0.20651  0.20525  0.20480  0.20418;
                                                        ];

% n_rf=50
olmpc = [  
         0.01141  0.01147  0.01145  0.01143  0.01141;
         0.02603  0.02648  0.02596  0.02604  0.02606;
         0.04754  0.04741  0.04906  0.04921  0.04793;
         0.07885  0.08011  0.07929  0.08052  0.08280;
         0.17438  0.17515  0.17352  0.17715  0.17601;  
                                                        ];


olmpc_indi = [  
         0.00660  0.00655  0.00666  0.00656  0.00647;
         0.01715  0.01712  0.01808  0.01763  0.01774;
         0.03541  0.03594  0.03548  0.03501  0.03604;
         0.06461  0.06235  0.06050  0.06274  0.06237;
         0.16394  0.15953  0.15225  0.14837  0.14479;
                                                        ];
                                                        
figure(4)
errorbar(max_speed_wrapped_lemniscate, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_lemniscate, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_lemniscate, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_wrapped_lemniscate, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.0 15.0]);
% title('Tracking Performance with wrapped Lemniscate Trajectory')
xlabel('Maximum Speed ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');                                                        



%% Tilted Circle
max_speed_tilted_circle = [2.5685 5.0936 7.6018 10.1162 12.6260];

mpc = [  
         0.06415  0.06398  0.06399  0.06398  0.06398;
         0.12408  0.12418  0.12418  0.12418  0.12417;
         0.17514  0.17515  0.17514  0.17515  0.17515;
         0.21774  0.21774  0.21773  0.21774  0.21774;
         0.26186  0.26186  0.26187  0.26191  0.26189;
                                                        ];


gpmpc = [  
         0.02925  0.02920  0.02920  0.02920  0.02920;
         0.04581  0.04590  0.04589  0.04590  0.04590;
         0.07212  0.07211  0.07211  0.07212  0.07212;
         0.11107  0.11106  0.11105  0.11106  0.11105;
         0.16093  0.16330  0.16327  0.16333  0.16334;
                                                        ];  

% n_rf=50
olmpc = [  
         0.01152  0.01165  0.01183  0.01165  0.01170;
         0.01740  0.01736  0.01770  0.01747  0.01758;
         0.02890  0.03361  0.03403  0.03411  0.03382;
         0.05296  0.05532  0.05057  0.05062  0.04994;
         0.09322  0.09863  0.09609  0.09457  0.09541;
                                                        ];

olmpc_indi = [  
         0.00308  0.00312  0.00318  0.00310  0.00308;
         0.00808  0.00887  0.00851  0.00870  0.00781;
         0.01845  0.02017  0.01871  0.01992  0.01984;
         0.03032  0.02592  0.02929  0.03191  0.03008;
         0.05715  0.06584  0.06456  0.06431  0.06350;
                                                        ];

figure(5)
errorbar(max_speed_tilted_circle, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
errorbar(max_speed_tilted_circle, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_tilted_circle, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
errorbar(max_speed_tilted_circle, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
xlim([2.0 13.0]);
% title('Tracking Performance with Tilted Circle Trajectory')
xlabel('Maximum Speed ($m/s$)','interpreter','latex');
ylabel('RMSE ($m$)','interpreter','latex');
legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');   



% %% wrapped Lemniscate (change in height: 1m)
% max_speed_wrapped_lemniscate = [3.4654 7.0941 10.5024 14.1096 17.7286];

% mpc = [  
%          0.05943  0.05944  0.05982  0.05981  0.05981;
%          0.11924  0.11944  0.11923  0.11944  0.11923;
%          0.16519  0.16532  0.16532  0.16516  0.16521;
%          0.20757  0.20754  0.20759  0.20745  0.20756;
%          0.28579  0.28565  0.28592  0.28559  0.28538;
%                                                         ];

% gpmpc = [  
%          0.02922  0.02940  0.02940  0.02921  0.02940;
%          0.04145  0.04146  0.04147  0.04157  0.04157;
%          0.07259  0.07256  0.07262  0.07259  0.07259;
%          0.11402  0.11406  0.11404  0.11404  0.11406;
%          0.20334  0.20313  0.20618  0.20652  0.20363;
%                                                         ]; 

% % n_rf=50
% olmpc = [  
%          0.01286  0.01314  0.01314  0.01324  0.01315;
%          0.02686  0.02654  0.02677  0.02649  0.02628;
%          0.04083  0.04252  0.04310  0.04121  0.04166;
%          0.05981  0.05991  0.06016  0.05916  0.06082;
%          0.16312  0.16260  0.16274  0.16268  0.16179;
%                                                         ];

% olmpc_indi = [  
%          0.00659  0.00682  0.00637  0.00651  0.00630;
%          0.01644  0.01521  0.01559  0.01521  0.01527;
%          0.02649  0.02547  0.02616  0.02749  0.02573;
%          0.03859  0.04002  0.03940  0.04003  0.03859;
%          0.14688  0.14286  0.14125  0.13803  0.14202;
%                                                         ];

% figure(6)
% errorbar(max_speed_wrapped_lemniscate, mean(mpc,2), std(mpc,0,2), 's-', 'Color', color_mpc, 'LineWidth', lw); hold on;
% errorbar(max_speed_wrapped_lemniscate, mean(gpmpc,2), std(gpmpc,0,2), 'o-', 'Color', color_gpmpc, 'LineWidth', lw); hold on;
% errorbar(max_speed_wrapped_lemniscate, mean(olmpc,2), std(olmpc,0,2), '^-', 'Color', color_olmpc, 'LineWidth', lw); hold on;
% errorbar(max_speed_wrapped_lemniscate, mean(olmpc_indi,2), std(olmpc_indi,0,2), '*-', 'Color', color_olmpc_indi, 'LineWidth', lw); hold on;
% xlim([2.5 20.0]);
% % title('Tracking Performance with wrapped Lemniscate Trajectory')
% xlabel('Maximum Speed ($m/s$)','interpreter','latex');
% ylabel('RMSE ($m$)','interpreter','latex');
% legend('Nominal MPC', 'GP-MPC', 'Ours', 'Ours (w/ INDI)', 'Location', 'northwest');  