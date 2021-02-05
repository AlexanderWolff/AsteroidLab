function number = bin2double(bin)

    % Separate Packet
    sign = bin(1);
    exp = bin(2:12);
    fraction = bin2dec(bin(13:64)');

    A = (-1)^sign;
    B  = 2^(bin2dec(exp)-1023);
    
    % Calculate Fraction
    sum = 0;
    for i = 1:52
        sum = sum+(fraction(i)*2^(-i));
    end

    C = (1+sum);
    
    % Convert
    number = A * B * C;

end