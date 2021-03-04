function A = CosineRule(a,b,c)
    % Find angle A by cosine rules
    A = acos((b^2+c^2-a^2)/(2*b*c));

end