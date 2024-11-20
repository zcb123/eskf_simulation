% 初始化数据
n = 500; % 点云大小
x = rand(1, n) * 10;
y = rand(1, n) * 10;
z = rand(1, n) * 10;

% 初始化绘图
figure;
h = scatter3(x, y, z, 20, z, 'filled'); % 使用颜色编码 Z 值
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Dynamic 3D Point Cloud');

% 动态更新点云
for t = 1:50
    x = rand(1, n) * 10;
    y = rand(1, n) * 10;
    z = rand(1, n) * 10;
    set(h, 'XData', x, 'YData', y, 'ZData', z, 'CData', z); % 更新点云数据
    drawnow;
    pause(0.1); % 模拟延迟
end
