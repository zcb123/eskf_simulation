function makeRowColSymmetric(width,first)

    global P;
    P_slice = P(first:first+width-1,first:first+width-1);
    if width > 1
        P(first:first+width-1,first:first+width-1) = (P_slice + P_slice')/2;
    end
end

