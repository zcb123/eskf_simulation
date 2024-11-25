function[f,y] = eu_fft(x,fs) 
    x_f = fft(x);           %计算信号的傅里叶变换
    
    len = length(x_f);
    %x = 1:len;
    
    %计算双侧频谱 P2。然后基于 P2 和偶数信号长度 L 计算单侧频谱 P1
    P2 = abs(x_f/len);
    
    P1 = P2(1:len/2+1);         %除第一个直流分量不用乘以2之外，别的信号都需要乘以2
    
    P1(2:end-1) = 2*P1(2:end-1);
    
    
    f = fs*(0:(len/2))/len;
    
    y = P1;
    
end

