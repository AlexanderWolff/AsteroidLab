function uint8v = vector6d2uint8(vector6d)

    vector6d = vector6d';
    uint8v = ones(1, length(vector6d)*8); 
    for i = 1:length(vector6d)

        buffer = bin2uint8_array(double2bin(vector6d(i)));
        uint8v( (i*8)-7: i*8) = buffer;
    end
end