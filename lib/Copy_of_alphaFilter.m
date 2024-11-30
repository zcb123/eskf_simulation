function res = alphaFilter(u,a)
    persistent filt_state;
    if isempty(filt_state)
        filt_state = zeros(length(u),1);
    end
    res = (1-a).*filt_state + a.*u;

end