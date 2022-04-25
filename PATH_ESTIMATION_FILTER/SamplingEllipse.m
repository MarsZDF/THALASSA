function [Points2] = SamplingEllipse(a, b, c, d, e, f, M)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
A = -sqrt(2*(a*(e^2) + c*(d^2) - b*d*e + (b^2 - 4*a*c)*f)*((a+c) + sqrt((a-c)^2 + b^2)))/(b^2 - 4*a*c);
B = -sqrt(2*(a*(e^2) + c*(d^2) - b*d*e + (b^2 - 4*a*c)*f)*((a+c) - sqrt((a-c)^2 + b^2)))/(b^2 - 4*a*c);
Cx = (2*c*d - b*e)/(b^2 - 4*a*c);
Cy = (2*a*e - b*d)/(b^2 - 4*a*c);

if b ~= 0
    Angle = atan((c - a - sqrt((a - c)^2 + b^2))/b); %atan2?
elseif b == 0 && a < c
    Angle = 0;
elseif b == 0 && a > c
    Angle = pi/2;
end

%For now it suffices; find better ways to space it
T = linspace(0, 2*pi, M);

%t is the parameter
for t = 1:M
    x(t) = A*cos(T(t))*cos(Angle) - B*sin(T(t))*sin(Angle) + Cx;
    y(t) = A*cos(T(t))*sin(Angle) + B*sin(T(t))*cos(Angle) + Cy;
end

Points2 = [x', y'];
hold on
%plot(Points2(:,1), Points2(:,2), 'LineWidth', 3, 'Color', '#EFC93D')
end

