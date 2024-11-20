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
