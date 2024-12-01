function setBaroData(baro_sample,baro_index)

global baro_sample_delayed;

    baro_sample_delayed.hgt = compensateBaroForDynamicPressure(baro_sample.Hight(baro_index,1));

end

