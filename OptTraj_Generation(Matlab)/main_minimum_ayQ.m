%% 生成参考轨迹
clear;clc;close all;
addpath('LG_TrajOpt');
X0.L = 0.72;
X0.time = 0.0;
X0.yQ = 0.0;
X0.dyQ = 0.0;
X0.ye = X0.L*sin(deg2rad(0));
X0.dye = X0.L*cos(deg2rad(0))*deg2rad(0);

X1.time = 5; 
X1.yQ = 2.0 - X0.L*sin(deg2rad(70));
X1.dyQ = 2.0;
X1.ye = X0.L*sin(deg2rad(70));
X1.dye = X0.L*cos(deg2rad(70))*deg2rad(0);

[Q1,P1,traj1] = LG_traj_min_ayQ(X0,X1);

simT = Q1.tic(end);
%% 轨迹动画
close all;
LG_ani( Q1,P1 );
%% 绘制轨迹中相关变量的曲线
figure(2)
subplot(221);plot(Q1.tic,Q1.ay,'r'),legend('ayQ');grid on;

subplot(222);plot(Q1.tic,Q1.y,'r');
hold on;plot(Q1.tic(end),Q1.y(end),'r*');legend('yQ');grid on;

subplot(223);plot(Q1.tic,Q1.dy,'r');
hold on;plot(Q1.tic(end),Q1.dy(end),'r*');legend('dyQ');grid on;

subplot(224);plot(Q1.tic,P1.ye,'r');
hold on;plot(Q1.tic(end),P1.ye(end),'r*');legend('ye');grid on;
%% 将轨迹数据保存在csv文件中:
%  需要跟踪的数据依次是：加速度、速度和位置

data = [Q1.ay,traj1(:,2),traj1(:,1)];% ay;dy;y
csvwrite('traj_data.csv',data);
%%