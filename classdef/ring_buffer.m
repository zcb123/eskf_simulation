classdef ring_buffer < handle
    properties
        elements
        len
        head
        tail
        first_write
    end

    methods
        %构造函数
        function obj = ring_buffer(maxlen)
            obj.len = maxlen;
            obj.elements = cell(maxlen,1);
            obj.head = 1;
            obj.tail = 1;
            obj.first_write = true;
        end
        %入队操作
        function obj = push(obj,sample)
            %if obj.
            head_new = obj.head;
            if ~obj.first_write
                %matlab 的索引从1开始到len，因此要先mod，后加1
                head_new = mod((obj.head),obj.len) + 1; 
            end
            obj.elements{head_new} = sample;
            obj.head = head_new;
            %判满
            if (obj.head == obj.tail && ~obj.first_write) 
                %如果满了，则写入最后一个元素，将最前面的元素覆盖，即mod(tail+1,len);
			    obj.tail = mod((obj.tail),obj.len) +1;
		    else 
			    obj.first_write = false;
            end
        end

        function [res,ret] = pop_first_older_than(obj,timestamp)
            for i = 1:obj.len
                index = obj.head - i + 1;
%                 if index <= 0
%                     index = index + 1;
%                 end
                % 有第一个元素最新的，后面的元素不是最新的情况，因此要循环遍历
                % index == 1时，刚好是第一个元素，此时index = index
                if index < 1
                    index = obj.len + index;     
%                 else
%                     index = index;    这个部分没有效果
                end

                if obj.elements{index}.time_us <= timestamp && timestamp < obj.elements{index}.time_us +1e5
                    res = obj.elements{index};
                    %
                    if index == obj.head    %不保留旧数据，清空队列
                        obj.tail = obj.head;
                        obj.first_write = true;
                    else
                        obj.tail = mod(index,obj.len) +1;   %取一个数据，tail+1
                    end

                    obj.elements{index}.time_us = 0;
                    ret = true;
                    return;
                end   
                %
                if index == obj.tail
                    ret = false;
                    return;
                end
            end
            ret = false;
        end

        function res = get_newest(obj)
            res = obj.elements{obj.head};
        end
        function res = get_oldest(obj)
            res = obj.elements{obj.tail};
        end
        %不判满，满了直接覆盖.只判空
        function ret = isEmpty(obj)
            ret = (obj.head ==obj.tail && obj.first_write == true);
        end
    end
end
