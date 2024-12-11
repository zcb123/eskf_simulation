function [res,val] = get_lookup_table_index(val_in,min,max)

    SAMPLING_RES = 10;

    val = saturation(val_in, min, max - SAMPLING_RES);

    res = (-(min) + val) / SAMPLING_RES + 1;        %matlab索引从1开始，这里加1

end