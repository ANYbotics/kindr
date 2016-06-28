function R = getRotationMatrixX(gamma)

c = cos(gamma);
s = sin(gamma);

if ~isa(gamma,'sym')
    if abs( c - round(c) ) < eps
        c = round(c);
    end

    if abs( s - round(s) ) < eps
        s = round(s);
    end
end

R = [1  0  0;
     0  c -s;
     0  s  c];

end