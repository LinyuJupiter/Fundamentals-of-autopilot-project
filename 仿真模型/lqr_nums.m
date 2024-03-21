clear;clc;
%% 此文件是仿真模型运行时加载的，需修改pathnums
pathnums='6';
load(['地图与路径/traj_diySYSU',pathnums,'.mat'])
load(['地图与路径/sysu_standard',pathnums,'.mat'])
m=0.041;%整车质量
a=0.4;%质心到前轴的距高
b=0.4;%质心到后轴的距高
L=a+b;%轴距
Iz=27.8e-6;%绕z轴的转动惯量
k1=-112600;%前轴侧偏刚度
k2=-89500;%后轴侧偏刚度
vx=3;%车速
A=[0,vx,1,0;
0,0,0,1;
0,0,(k1+k2)/m/vx, (a*k1-b*k2)/m/ vx-vx;
0,0,(a*k1-b*k2)/Iz/vx, (a^2*k1+b^2*k2)/Iz/vx];
B=[0;0; k1/m; -a*k1/Iz];
Q=[10,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
R=1;
k=lqr(A,B,Q,R);