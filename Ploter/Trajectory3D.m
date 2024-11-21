% 创建数据
t = linspace(0, 10, 100); % 时间点
x = t;
y = sin(t);
z = cos(t);

% 初始化绘图
figure;
h = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5); % 初始化轨迹线条
hold on;
p = plot3(nan, nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % 初始化移动点
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Dynamic 3D Trajectory');

% 动态更新轨迹和点的位置
for i = 1:length(t)
    set(h, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z(1:i)); % 更新轨迹
    set(p, 'XData', x(i), 'YData', y(i), 'ZData', z(i));       % 更新点的位置
    drawnow;
    pause(0.05); % 模拟延迟
end


%% 动态绘制三维轨迹


figure;
h = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5); % 初始化轨迹线条
hold on;
p = plot3(nan, nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % 初始化移动点
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Dynamic 3D Trajectory');
set(h, 'XData', pos_new(1:i,1), 'YData', pos_new(1:i,2), 'ZData', pos_new(1:i,3)); % 更新轨迹
set(p, 'XData', pos_new(i,1), 'YData', pos_new(i,2), 'ZData', pos_new(i,3));       % 更新点的位置
drawnow;
%%

%     persistent figHandle; % 使用持久变量存储 figure 句柄
% 
%     if isempty(figHandle) || ~isvalid(figHandle)
%         figHandle = figure; % 如果句柄不存在或无效，创建新 figure
%     else
%         figure(figHandle); % 如果句柄存在，激活 figure
%     end
    
    figure(1); % 激活目标 figure
    %clf; % 清空当前 figure 的内容
    plot(imu.time_us,delta_angle(1),'b', 'LineWidth', 2);
    grid on