clear; clc;
%% 此文件是将缩小后路径平滑化和曲率筛选后，处理成输入仿真模型所需结构的，需修改pathnums
pathnums='6';
load(['地图与路径/path',pathnums,'.mat'])
load(['地图与路径/sysu_standard',pathnums,'.mat'])
ormap=map;
map = ~map;
se=strel('disk',11);
ormap=~imdilate(ormap,se);
path(:, 1:2) = path(:, 1:2) * 5;
%motionAngles = [0, pi/4, -pi/4, pi/2, -pi/2, pi, 3*pi/4, -3*pi/4];
%path(:, 3) = motionAngles(path(:, 3));

% 起始点和目标点
startX = 520;
startY = 80;
goalX = 150;
goalY = 1130;

% 原始轨迹点
rawPath = path(:, 1:2); % 只使用轨迹的 x 和 y 坐标
rawPath= [[startX,startY+20];rawPath];
% 插值处理参数
interpolationFactor = 5; % 插值因子，控制插值后的数据点数量
% 对原始路径进行插值
interpolatedPath = interp1(1:size(rawPath, 1), rawPath, linspace(1, size(rawPath, 1), interpolationFactor*size(rawPath, 1)));
% 平滑处理参数
smoothness = 1; % 调整平滑程度，范围为 0 到 1

% 地图大小
[mapWidth, mapHeight] = size(map);

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

% 绘制路径
for i = 1:size(path, 1)
    x = path(i, 1);
    y = path(i, 2);
    mapImage(x, y, :) = [0 0 1]; % 蓝色表示路径上的点
end

smoothPathFixed1=SegmentedSmoothing(interpolatedPath, ormap,512,smoothness);
smoothPathFixed1=curvatureFilter(smoothPathFixed1,4,ormap);
smoothPathFixed2=SegmentedSmoothing(smoothPathFixed1, ormap,512,smoothness);
smoothPathFixed2=curvatureFilter(smoothPathFixed2,4,ormap);
smoothPathFixed3=SegmentedSmoothing(smoothPathFixed2, ormap,512,smoothness);
smoothPathFixed3=curvatureFilter(smoothPathFixed3,4,ormap);
smoothPathFixed=SegmentedSmoothing(smoothPathFixed3, ormap,512,smoothness);
trajSYSU=get_trajSYSU(smoothPathFixed,ormap);

% 显示地图图像
imshow(mapImage);
hold on;

% 绘制原始轨迹和平滑后的轨迹
plot(interpolatedPath(:, 2), interpolatedPath(:, 1), 'r', 'LineWidth', 2);
hold on;
% plot(smoothPathFixed1(:, 2), smoothPathFixed1(:, 1), 'b', 'LineWidth', 2);
% hold on;
% plot(smoothPathFixed2(:, 2), smoothPathFixed2(:, 1), 'g', 'LineWidth', 2);
% hold on;
% plot(smoothPathFixed3(:, 2), smoothPathFixed3(:, 1), 'y', 'LineWidth', 2);
% hold on;
plot(smoothPathFixed(:, 2), smoothPathFixed(:, 1), 'k', 'LineWidth', 2);
legend('原始轨迹', '第四次平滑后的轨迹');
xlabel('X');
ylabel('Y');
axis equal;
save(['地图与路径/traj_diySYSU',pathnums,'.mat'],'trajSYSU')
%save(['仿真模型/地图与路径/smoothPath',pathnums,'.mat'],'smoothPathFixed');
%% 使用贝塞尔曲线进行平滑处理的函数
function smoothPath = smoothBezier(path, smoothness)
    n = size(path, 1);
    if n < 3
        smoothPath = path; % 路径点数量小于 3，无法进行平滑处理，直接返回原始路径
        return;
    end
    
    % 计算控制点
    controlPoints = zeros(n + 2, 2);
    controlPoints(2:n+1, :) = path;
    
    % 添加两个虚拟控制点
    controlPoints(1, :) = 2 * path(1, :) - path(2, :);
    controlPoints(n+2, :) = 2 * path(n, :) - path(n-1, :);
    
    % 计算平滑后的路径
    t = linspace(0, 1, smoothness * (n - 1) + 2);
    smoothPath = bezierCurve(controlPoints, t);
    smoothPath(1,:)=path(1,:);
    smoothPath(end,:)=path(end,:);
end

%% 贝塞尔曲线插值函数
function curve = bezierCurve(controlPoints, t)
    curve = zeros(length(t), 2);
    
    for i = 1:length(t)
        curve(i, :) = deCasteljau(controlPoints, t(i));
    end
end

%% De Casteljau 算法
function point = deCasteljau(controlPoints, t)
    n = size(controlPoints, 1) - 1;
    points = controlPoints;
    
    for r = 1:n
        for i = 1:n-r+1
            points(i, :) = (1 - t) * points(i, :) + t * points(i+1, :);
        end
    end
    
    point = points(1, :);
end

%% 分段平滑化并处理碰撞问题
function smoothPathFixed =SegmentedSmoothing(path,map,Scoring,smoothness)
if Scoring<=2
    smoothPathFixed=path;
else
    % 分段平滑化并处理碰撞问题
    smoothPathSegments = cell(1, size(path, 1)-1); % 存储分段平滑化后的路径段

    for i = 1:ceil(size(path, 1)/Scoring)
        % 提取当前路径段
        segment = path(Scoring*i-Scoring+1:min(Scoring*i,size(path, 1)), 1:2);
        % 平滑化当前路径段
        segmentSmooth = smoothBezier(segment, smoothness);
        % 检查平滑路径段与地图中的障碍物的碰撞
        collision = false;
        for j = 1:size(segmentSmooth, 1)
            x = round(segmentSmooth(j, 1));
            y = round(segmentSmooth(j, 2));
            if map(x, y) == 0
                collision = true;
                break;
            end
        end
        % 如果碰撞，则进一步细分路径段再进行平滑化
        if collision
            segmentSmooth = SegmentedSmoothing(segment, map,round(Scoring/2),smoothness);
        end

        % 存储平滑化后的路径段
        smoothPathSegments{i} = segmentSmooth;
    end

    % 将平滑路径段拼接起来得到最终的平滑路径
    smoothPathFixed = cat(1, smoothPathSegments{:});
end
end
%% 曲率过滤函数
function filteredPath = curvatureFilter(path, maxCurvature,map)
    truepath=zeros(size(path,1),size(path,2)+1);
    [~,ro]=get_trajSYSU(path,map);
    truepath(:,1:2)=path(:,1:2);
    truepath(:,3)=ro;
    fakepath=truepath;
    filteredPath=truepath;
    i=1;
    while i~=size(fakepath,1)
        if fakepath(i,3)>=maxCurvature
            fakepath(i,:)=[];
            % 检查平滑路径段与地图中的障碍物的碰撞
            collision = false;
            for j = 1:size(fakepath, 1)
                x = round(fakepath(j, 1));
                y = round(fakepath(j, 2));
                if map(x, y) == 0
                    collision = true;
                    break;
                end
            end
            % 如果碰撞，则返还
            if collision
                fakepath=filteredPath;
                i=i+1;
            else
                filteredPath=fakepath; 
            end
        elseif fakepath(min(i+1,size(fakepath,1)),3)>=maxCurvature
            fakepath(i+1,:)=[];
            % 检查平滑路径段与地图中的障碍物的碰撞
            collision = false;
            for j = 1:size(fakepath, 1)
                x = round(fakepath(j, 1));
                y = round(fakepath(j, 2));
                if map(x, y) == 0
                    collision = true;
                    break;
                end
            end
            % 如果碰撞，则返还
            if collision
                fakepath=filteredPath;
                i=i+1;
            else
                filteredPath=fakepath; 
            end
        else
            i=i+1;
        end
        
    end
end
%% get_trajSYSU 处理数据格式
function [trajSYSU,ro]=get_trajSYSU(smoothPathFixed,map)
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
end