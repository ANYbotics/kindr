function R = getRotationMatrixY(beta)

c = cos(beta);
s = sin(beta);

if ~isa(beta,'sym')
    if abs( c - round(c) ) < eps
        c = round(c);
    end

    if abs( s - round(s) ) < eps
        s = round(s);
    end
end

R = [c  0  s;
     0  1  0;
    -s  0  c];

end