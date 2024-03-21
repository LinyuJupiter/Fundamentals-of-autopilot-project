% subplot(1,2,1)
% imshow(~ormap)
% 
% se=strel('disk',11);
% dilated_map=~imdilate(ormap,se);
% subplot(1,2,2)
% imshow(dilated_map)
% 创建地图的可视化图像
% 回溯生成路径
path = [];
currentNode = closedList(end);
while ~isempty(currentNode.parent)
    path = [currentNode.x, currentNode.y, currentNode.phi; path];
    currentNode = currentNode.parent;
end
% load('path.mat')

mapImage = ones(mapWidth, mapHeight, 3); % 白色表示可通行区域
mapImage(~map == 0) = 0; % 将障碍物和墙标记为黑色

% 绘制起始点和目标点
aa=round(15*mapWidth/600);
mapImage(startX-aa:startX+aa, startY-aa:startY+aa, 1)=0;
mapImage(startX-aa:startX+aa, startY-aa:startY+aa, 2) = 1; % 绿色表示起始点
mapImage(startX-aa:startX+aa, startY-aa:startY+aa, 3)=0;
mapImage(goalX-aa:goalX+aa, goalY-aa:goalY+aa, 1)=1;
mapImage(goalX-aa:goalX+aa, goalY-aa:goalY+aa, 2) = 0; % 绿色表示起始点
mapImage(goalX-aa:goalX+aa, goalY-aa:goalY+aa, 3)=0;

% 绘制路径
for i = 1:size(path, 1)
    x = path(i, 1);
    y = path(i, 2);
    mapImage(x, y, :) = [0 0 1]; % 蓝色表示路径上的点
end

% 显示地图图像
imshow(mapImage);
hold on;
for i =1:size(path,1)-1
    line([path(i,2) path(i+1,2)], [path(i,1) path(i+1,1)], 'Color', 'b')
end