function [lambda] = FindLambdar(r_x,r_y,r_z,a,b,c)
%FindLambdar this function find the maximal real root of a cubic polynomial
% In:
% r_x, r_y, r_z Position in orbit
% a, b, c Asteroid radii
%
% Out:
% Lambda
% Description:
% This function find the solution to a second order equation, the output
% value lambda is used in the calculation of the elliptic integral in rj
sva=a^2;
svb=b^2;
svc=c^2;
svx=r_x^2;
svy=r_y^2;
svz=r_z^2;
%Polynomial coef
n0 = (sva*svb*svc)-(sva*svb*svz+sva*svc*svy+svc*svb*svx);
n1 = (sva*svb+sva*svc+svb*svc)-((sva+svb)*svz+(sva+svc)*svy+(svc+svb)*svx);
n2 = (sva+svb+svc)-(svx+svy+svz);
n3 = 1;
%Roots
r = roots([n3 n2 n1 n0]);
r = r(imag(r)==0); % Save only the real roots
lambda = max(r); %Save the maximum real root
end