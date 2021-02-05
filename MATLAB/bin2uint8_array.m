function uint8_array = bin2uint8_array(bin)

    len = length(bin);
    if mod(len,8) == 0

        uint8_array = zeros(1,len/8);

        for i = 1:(len/8)

            uint8_array(i) = bin2dec(bin( (i*8)-7:i*8 ));

        end
    end

end