function quat_out = quat_normalize(quat_in)
    len = norm(quat_in)
    quat_out = [quat_in(1)/len quat_in(2)/len quat_in(3)/len quat_in(4)/len];

end