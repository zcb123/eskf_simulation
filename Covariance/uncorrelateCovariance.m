function uncorrelateCovariance(width,first)
    global P;
    P_Slice = P(first:first+width,first:first+width);
    diag_elements = diag(P_Slice);
    uncorrelateCovarianceSetVariance(width,first,diag_elements);


end
