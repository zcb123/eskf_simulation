
%是否有其他航向源已经停止了
function result = otherHeadingSourcesHaveStopped()

    global non_mag_yaw_aiding_running_prev
    
    result = noOtherYawAidingThanMag() && non_mag_yaw_aiding_running_prev;

    non_mag_yaw_aiding_running_prev = ~noOtherYawAidingThanMag();


end