%输出-pi~pi
function y = wrap_pi(u)
    len = length(u);
    for i = 1:len
        res = mod(u(i),2*pi);               %输出0~2*pi

        if res > pi
            res = res - 2*pi;               %输出-pi~pi
        end

        u(i) = res;
    end
    y = u;
end