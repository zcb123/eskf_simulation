% 创建数据
n = 100;
x = linspace(0, 10, n);
y = sin(x);
z = cos(x);

% 初始化绘图
figure;
h = plot3(nan, nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Animation');
filename = '3D_animation.gif';

% 动态更新并保存帧
for i = 1:n
    set(h, 'XData', x(i), 'YData', y(i), 'ZData', z(i)); % 更新点的位置
    drawnow;
    
    % 捕获当前帧并保存为 GIF
    frame = getframe(gcf);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end
