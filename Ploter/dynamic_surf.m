% 创建动态数据
[X, Y] = meshgrid(-5:0.1:5, -5:0.1:5);

% 初始化图形窗口
figure;
for t = 0:0.1:2*pi
    Z = sin(sqrt(X.^2 + Y.^2) + t); % 随时间变化的 Z 数据
    surf(X, Y, Z); % 绘制表面图
    shading interp; % 平滑着色
    colormap(jet);
    title(['动态表面图，时间 t = ', num2str(t, '%.2f')]);
    xlabel('X 轴');
    ylabel('Y 轴');
    zlabel('Z 轴');
    colorbar;
    pause(0.1); % 暂停 0.1 秒，形成动画效果
end
