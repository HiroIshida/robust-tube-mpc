function str = number2string(n)
    if n < 10
        str = strcat('0', string(n));
    else
        str = string(n);
    end
end
