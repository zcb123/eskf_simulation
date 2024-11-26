%%默认输入n行m列的val
function res = getNonNaN(val,m)

    nonNaN_index = ~isnan(val);     %获取非NaN元素的索引
    nonNaN_num = round(sum(nonNaN_index(:))/m);
    
    res = reshape(val(nonNaN_index),[nonNaN_num,m]);
%     res = reshape(val(nonNaN_index),[]);
end