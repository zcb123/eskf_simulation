function increaseQuatYawErrVariance(yaw_variance)
    global P;
    yaw_variance = min(yaw_variance, 1.0e-2);
    P(3,3) = P(3,3) + yaw_variance;
end