clear;clc;
%% 此文件是将仿真结果处理成GIF图的，需修改pathnums
pathnums='6';
load(['结果展示/out_simout',pathnums,'.mat'])
alldata=out;
load(['地图与路径/traj_diySYSU',pathnums,'.mat'])
load(['地图与路径/sysu_standard',pathnums,'.mat'])
x_data=alldata.simout.X;
y_data=alldata.simout.Y;
phi_data=alldata.simout.psi_unwrap;
numRows = size(x_data.Data,1);
runtime=numRows*0.001;
x=x_data.Data;
y=y_data.Data;
phi=phi_data.Data;
%figure
proportion=5;% 比例因子
s=0;
for times=1:round(numRows/proportion)
    map_now=display_map_obs_car_(x(times*proportion-proportion+1),y(times*proportion-proportion+1),phi(times*proportion-proportion+1),map,out,trajSYSU);
    if mod(times,20)==1
        %imshow(map_now)
        s=s+1;
        [I,maps] = rgb2ind(map_now,256);
        if(s==1)
            imwrite(I,maps,['结果展示/',pathnums,'--',num2str(runtime),'.gif'],'DelayTime',0.1,'LoopCount',Inf)
        else
            imwrite(I,maps,['结果展示/',pathnums,'--',num2str(runtime),'.gif'],'WriteMode','append','DelayTime',0.1)
        end
    end
end
map_now=display_map_obs_car_(x(numRows),y(numRows),phi(numRows),map,out,trajSYSU);
s=s+1;
[I,maps] = rgb2ind(map_now,256);
imwrite(I,maps,['结果展示/',pathnums,'--',num2str(runtime),'.gif'],'WriteMode','append','DelayTime',0.1)
imwrite(map_now,['结果展示/',pathnums,'--',num2str(runtime),'.png'])
%imshow(map_now)

%% 展示地图
function out = display_map_obs_car_(x,y,phi,map_from_initcallback,matrix_for_viewer,traj_sysu)
persistent data;
persistent times;
if isempty(data)
    times=1;
else
    times=times+1;
end

% 地图长12m 宽6m   车长0.12 车宽0.06
resolution = 0.01;
map_long=12;
map_wide=6;

car_long=0.12;
car_wide=0.06;
map=map_from_initcallback;
out=matrix_for_viewer;
number_path=size(traj_sysu,2);
for tmp=1:number_path
    traj_sysu_index=real2map([traj_sysu(1,tmp),traj_sysu(2,tmp)],map_wide,resolution);
    out(round(traj_sysu_index(1)+1/2),round(traj_sysu_index(2)+1/2),:)=[0,0,0];
end
x_now=x;%m  real视野坐标系,横着往右
y_now=y;%m  real视野坐标系,竖着往上
map_coord=real2map([x_now,y_now],map_wide,resolution);
if isempty(data)
    data = zeros(99999, 2);
else
    data(round(times/10)+1,:) = [map_coord(1),map_coord(2)];
    for times_disp_tmp=1:round(times/10)
        tmp_Data=data(times_disp_tmp,:);
        tmp_Data_next=data(times_disp_tmp+1,:);
        %             out(round(tmp_Data(1,1)+1/2),round(tmp_Data(1,2)+1/2),:)=[0,0,1];


        vector=tmp_Data_next-tmp_Data;
        for t=0:0.025:1
            new_point=tmp_Data+t*vector;
            out(round(new_point(1)+1/2),round(new_point(2)+1/2),:)=[0,0,1];
            out(round(new_point(1)+1/2),round(new_point(2)+1/2)-1,:)=[0,0,1];
            out(round(new_point(1)+1/2),round(new_point(2)+1/2)+1,:)=[0,0,1];
        end
    end
end
l = car_long / resolution;
w = car_wide / resolution;
phi = phi; 
out=display_car(map_coord,phi,l,w,map,out);
end
%% 转换坐标系
function map_coord=real2map(x_y,map_wide,resolution)
    %地图向下是正x 向右是正y，
    % 因此在map上显示赛道小车障碍物的时候map_y直接取real世界的x, map_x需要把real_y反向并弄个静差
    map_coord=zeros(1,2);
    map_coord(1)=map_wide/ resolution-x_y(2)/ resolution;
    map_coord(2)=x_y(1)/ resolution ;
end
%% 展示车
function matrix_for_viewer_new=display_car(map_coord,phi,l,w,map,matrix_for_viewer)
    %display是向右是x正，向下是y正
    pts = [
        map_coord(1)- [sin(phi)*l-cos(phi)*w, sin(phi)*l+cos(phi)*w,-sin(phi)*l+cos(phi)*w,-sin(phi)*l-cos(phi)*w]
        map_coord(2) + [cos(phi)*l+sin(phi)*w, cos(phi)*l-sin(phi)*w,-cos(phi)*l-sin(phi)*w,-cos(phi)*l+sin(phi)*w]
        
    ];
    map_index1=[round(pts(1,1)+1/2),round(pts(2,1)+1/2)];
    map_index2=[round(pts(1,2)+1/2),round(pts(2,2)+1/2)];
    map_index3=[round(pts(1,3)+1/2),round(pts(2,3)+1/2)];
    map_index4=[round(pts(1,4)+1/2),round(pts(2,4)+1/2)];

    matrix_for_viewer_new=matrix_for_viewer;
    %凸包，边缘线无碰撞则整体无碰撞
    have_somewhere_occ=0;
    vector_A_B=[map_index2(1)-map_index1(1),map_index2(2)-map_index1(2)];
    vector_A_D=[map_index4(1)-map_index1(1),map_index4(2)-map_index1(2)];
    for t1=0:0.025:1
        for t2=0:0.025:1
            new_point=map_index1+t1*vector_A_B+t2*vector_A_D;
            matrix_for_viewer_new(round(new_point(1)+0.5),round(new_point(2)+0.5),:)=[1,0,0];
            occ_or_not=map(round(new_point(1)+0.5),round(new_point(2)+0.5));
            if(occ_or_not==1)
               have_somewhere_occ=1;
           end
        end
    end
    if(have_somewhere_occ==1)
%         disp("collision!!");
    end
end