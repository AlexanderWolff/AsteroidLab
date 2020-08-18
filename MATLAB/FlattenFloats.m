function [flatten] = FlattenFloats(floats)

    flatten = num2str(floats(1,:));
    len = size(floats,1);
    for i = 2:len
        flatten = strcat(flatten, ',' ,num2str(floats(i,:)));  
    end
end