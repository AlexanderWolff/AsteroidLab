function bin = double2bin(number)

    % sign bit (1 if negative)
    sign = dec2bin(number<0,1);

    % work out exponent (11 bits)
    n = abs(number);
    exp = 0;
    if n >= 1
        for i = 0:1023
           if n/2^i < 2
               exp = i+1023;
               break;
           end
        end
    else
        for i = 1:1023
            if n/2^-i > 1
               exp = -i+1023;
               break;
           end
        end
    end
    reminder = n/2^(exp-1023)-1;
    exp = dec2bin(exp,11);

    % work out fraction (52 bits)
    fraction = dec2bin(0,52);
    for i = 1:52

       if reminder - (2^-i) >= 0

          reminder = reminder - (2^-i);
          fraction(i) = '1';

          if reminder == 0
              break
          end
       end
    end

    bin = [sign, exp, fraction];

end
