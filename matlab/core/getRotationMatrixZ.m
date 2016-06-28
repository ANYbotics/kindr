function R = getRotationMatrixZ(alpha)

c = cos(alpha);
s = sin(alpha);

if ~isa(alpha,'sym')
    if abs( c - round(c) ) < eps
        c = round(c);
    end

    if abs( s - round(s) ) < eps
        s = round(s);
    end
end

R = [c -s  0;
     s  c  0;
     0  0  1];

end