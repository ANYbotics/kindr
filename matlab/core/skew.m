function S = skew(v)
% SKEW(v) outputs the skew matrix of a 3d vector v.
%
% Author(s): Dario Bellicoso

S = [ 0     -v(3)   v(2);
      v(3)   0     -v(1);
     -v(2)   v(1)   0  ];

end