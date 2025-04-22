clc; close all; clear;


lw = 1.5;
fs = 6;
ms = 8;
color_ref = 'k';
color_proj = [0.7 0.7 0.7];


load("OLMPC_circle_10.0_50.mat");
x_circle = ref_x;

load("OLMPC_lemniscate_10.0_50.mat");
x_lemniscate = ref_x;

load("OLMPC_wraped_circle_10.0.mat");
x_wraped_circle = ref_x;

load("OLMPC_wraped_lemniscate_10.0.mat");
x_wraped_lemniscate = ref_x;

figure(1)
plot3(x_circle(:,1),x_circle(:,2),x_circle(:,3), 'LineWidth', lw); hold on; 
plot3(x_circle(:,1),x_circle(:,2),zeros(size(x_circle(:,3))),'Color',color_proj,'LineWidth', lw); hold on;
grid on;
zlim([0, 3.5])
xlabel('${x}$ \ ($m$)','interpreter','latex');
ylabel('${y}$ \ ($m$)','interpreter','latex');
zlabel('${z}$ \ ($m$)','interpreter','latex');



figure(2)
plot3(x_wraped_circle(:,1),x_wraped_circle(:,2),x_wraped_circle(:,3), 'LineWidth', lw); hold on;
plot3(x_wraped_circle(:,1),x_wraped_circle(:,2),zeros(size(x_wraped_circle(:,3))),'Color',color_proj,'LineWidth', lw); hold on; 
grid on;
zlim([0, 4.5])
xlabel('${x}$ \ ($m$)','interpreter','latex');
ylabel('${y}$ \ ($m$)','interpreter','latex');
zlabel('${z}$ \ ($m$)','interpreter','latex');


figure(3)
plot3(x_lemniscate(:,1),x_lemniscate(:,2),x_lemniscate(:,3), 'LineWidth', lw); hold on; 
plot3(x_lemniscate(:,1),x_lemniscate(:,2),zeros(size(x_lemniscate(:,3))),'Color',color_proj,'LineWidth', lw); hold on; 
grid on;
zlim([0, 3.5])
xlabel('${x}$ \ ($m$)','interpreter','latex');
ylabel('${y}$ \ ($m$)','interpreter','latex');
zlabel('${z}$ \ ($m$)','interpreter','latex');


figure(4)
plot3(x_wraped_lemniscate(:,1),x_wraped_lemniscate(:,2),x_wraped_lemniscate(:,3), 'LineWidth', lw); hold on; 
plot3(x_wraped_lemniscate(:,1),x_wraped_lemniscate(:,2),zeros(size(x_wraped_lemniscate(:,3))),'Color',color_proj,'LineWidth', lw); hold on; 
grid on;
zlim([0, 4.5])
xlabel('${x}$ \ ($m$)','interpreter','latex');
ylabel('${y}$ \ ($m$)','interpreter','latex');
zlabel('${z}$ \ ($m$)','interpreter','latex');

clear;