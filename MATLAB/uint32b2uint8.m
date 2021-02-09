function num = uint32b2uint8(input)

    num = bin2uint8_array(dec2bin(input,32));

end