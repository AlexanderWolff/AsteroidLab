function number = bin2uint16(bin)

    % Concatenate Binary Strings
    bin = reshape(bin',1,[]);
    
    % Convert
    number = bin2dec(bin);
    
end