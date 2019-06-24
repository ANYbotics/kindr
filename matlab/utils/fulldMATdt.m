% function [ ddA ] = d2MATdt2(  A, n, q, dq, ddq)
%
% -> nth time derivation of a matrix or a vector A(q).
% dnA(q)/dtn
%
% Note that this function was only proved up to second derivative.
%
% Created by Christian Gehring on 11.10.2012
% from Matlab2010a
% -> cgehring@anybotics.com

function [ ddA ] = fulldMATdt( A, n, q, dq, ddq)
if n > 2
    error('This function was not checked for higher derivatives!')
end
ddA = fulldiff2(A,num2cell(q),n,num2cell(dq),num2cell(ddq));
end
