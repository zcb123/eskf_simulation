%t的单位是微秒
function[f,y] = eust_fft(t,x)
    dt = diff(t);
    dt_a = mean(dt);
    fs = 1/(dt_a * (1e-6));
    [f,y]=eu_fft(x,fs);
end



