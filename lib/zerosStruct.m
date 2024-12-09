function struct_out = zerosStruct(struct_in)

    fields = fieldnames(struct_in);
    for i = 1:numel(fields)
        struct_in.(fields{i}) = zeros(size(struct_in.(fields{i})));
    end
    struct_out = struct_in;
end