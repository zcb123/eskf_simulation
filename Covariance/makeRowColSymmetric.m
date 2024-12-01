function makeRowColSymmetric(width,first)

    global P;
    P_slice = P(first:first+width,first:first+width);
    if width > 1
        P(first:first+width,first:first+width) = (P_slice + P_slice')/2;
    end
end

