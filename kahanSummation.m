function [t,accumlator_err] = kahanSummation(sum_previous,input,accumulator)

    y = input - accumulator;
    t = sum_previous + y;		% t = sum_previous + input - accumulator
	accumlator_err = (t - sum_previous) - y;	% accumulator = sum_previous + input - accumulator - sum_previous - input + accumulator 若不等于0，则表明有舍入误差，在下一次累加中补偿

end