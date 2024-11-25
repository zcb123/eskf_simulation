function[f,y] = eu_fft(x,fs) 
    x_f = fft(x);           %�����źŵĸ���Ҷ�任
    
    len = length(x_f);
    %x = 1:len;
    
    %����˫��Ƶ�� P2��Ȼ����� P2 ��ż���źų��� L ���㵥��Ƶ�� P1
    P2 = abs(x_f/len);
    
    P1 = P2(1:len/2+1);         %����һ��ֱ���������ó���2֮�⣬����źŶ���Ҫ����2
    
    P1(2:end-1) = 2*P1(2:end-1);
    
    
    f = fs*(0:(len/2))/len;
    
    y = P1;
    
end

