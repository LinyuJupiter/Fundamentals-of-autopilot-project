clear;clc;
pathnums='3';
load(['仿真模型/地图与路径/smoothPath',pathnums,'.mat'])
load(['仿真模型/地图与路径/sysu_standard',pathnums,'.mat'])
map = ~map;
% 地图大小
[mapWidth, mapHeight] = size(map);

% 起始点和目标点
startX = 520;
startY = 80;
goalX = 150;
goalY = 1130;
trajSYSU=zeros(size(smoothPathFixed',1)+2,size(smoothPathFixed',2));
for i=1:size(smoothPathFixed,1)
    trajSYSU(2,i)=mapWidth-smoothPathFixed(i,1);
    trajSYSU(1,i)=smoothPathFixed(i,2);
end
trajSYSU=trajSYSU/100;
for i=2:size(trajSYSU,2)
    vec=trajSYSU(:,i)-trajSYSU(:,i-1);
    angle=atan2(vec(2),vec(1));
    trajSYSU(3,i-1)=angle;
end
trajSYSU(3,end)=0;
x=trajSYSU(1,:);
y=trajSYSU(2,:);
x1=[0,diff(x)];
y1=[0,diff(y)];
x2=[0,diff(x1)];
y2=[0,diff(y1)];
ro=(x1.*y2-x2.*y1)./(x1.^2+y1.^2).^(3/2);
ro(1)=0;
trajSYSU(4,:)=ro;
save(['仿真模型/地图与路径/traj_diySYSU',pathnums,'.mat'],'trajSYSU')
mapImage = ones(mapWidth, mapHeight, 3); % 白色表示可通行区域
mapImage(map == 0) = 0; % 将障碍物和墙标记为黑色

% 绘制起始点和目标点
aa = 15;
mapImage(startX - aa:startX + aa, startY - aa:startY + aa, 1) = 0;
mapImage(startX - aa:startX + aa, startY - aa:startY + aa, 2) = 1; % 绿色表示起始点
mapImage(startX - aa:startX + aa, startY - aa:startY + aa, 3) = 0;
mapImage(goalX - aa:goalX + aa, goalY - aa:goalY + aa, 1) = 1;
mapImage(goalX - aa:goalX + aa, goalY - aa:goalY + aa, 2) = 0; % 红色表示目标点
mapImage(goalX - aa:goalX + aa, goalY - aa:goalY + aa, 3) = 0;
% 显示地图图像
imshow(mapImage);
hold on;

plot(smoothPathFixed(:, 2), smoothPathFixed(:, 1), 'b', 'LineWidth', 2);
legend('平滑后的轨迹');
xlabel('X');
ylabel('Y');
axis equal;