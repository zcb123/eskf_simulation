function uncorrelateCovariance(width,first)
    global P;
    P_Slice = P(first:first+width-1,first:first+width-1);
    diag_elements = diag(P_Slice);
    uncorrelateCovarianceSetVariance(width,first,diag_elements);


end
