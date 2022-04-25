function [x0] = x0_CircularRandom
%Randomly generates the initial state vector to achieve a Circular Orbit:
%[1] https://stackoverflow.com/questions/14845273/initial-velocity-vector-for-circular-orbit
G = 6.674E-11; %Gravitational Parameter
global Ancillary 
Rx = Ancillary(1); Ry = Ancillary(2); Rz = Ancillary(3); Rho = Ancillary(4); Om = Ancillary(5);
RAx = Ancillary(6:8); RVe = Om*RAx;
Volume = 4*pi*Rx*Ry*Rz/3;
mu = G*Volume*Rho; %standard gravitational parameter, ?
%The norm of the initial radius is set to 10 times the average of the
%radii.
R = [Rx, Ry, Rz]; RMagnitude = 5*mean(R); 
Rmax = max(R); Rcontrol = sum(R) - Rmax;
if 2*Rcontrol < Rmax
    error('The periapsis is smaller than the largest radius!')
end
%Random initial angular coordinates of the initial position
%https://mathworld.wolfram.com/SpherePointPicking.html
Theta = 2*pi*rand;     Phi = acos(2*rand - 1); 
%Initial position vector: spherical coordinates to Cartesian 
[x0_r(1), x0_r(2), x0_r(3)] = sph2cart(Theta, Phi, RMagnitude);
%Obtain the magnitude of velocity from the circular orbit constraint
Vnorm = sqrt(mu/norm(x0_r));
%e1 is perpendicular to the initial radius [1]; e2 is perpendicular to the
%initial radius and e1
e1 = [0, -x0_r(3), x0_r(2)]; e2 = cross(x0_r, e1);
%n1 and n2 represent the normalized e1 and e2
n1 = e1/norm(e1); n2 = e2/norm(e2);
%Define a random angle and use it to generate a random velocity vector 
%by projection on the new axes (orthogonal to the radius)
OMEga = rand*2*pi;
vx0 = cos(OMEga)*n1(1) + sin(OMEga)*n2(1); vy0 = cos(OMEga)*n1(2) + sin(OMEga)*n2(2); vz0 = cos(OMEga)*n1(3) + sin(OMEga)*n2(3);
%Set the velocity norm to sqrt(?/R)
x0_v = [vx0, vy0, vz0]*Vnorm/norm([vx0, vy0, vz0]);
%Update initial conditions: from inertial to rotating frame
x0_v2 = x0_v - cross(RVe, x0_r);
x0 = [x0_r, x0_v2]; 
h = [x0(2)*x0(6) - x0(3)*x0(5), x0(3)*x0(4) - x0(1)*x0(6), x0(1)*x0(5) - x0(2)*x0(4)]; i = rad2deg(acos(h(3)/norm(h)))
end

%Code written by M. Z. Di Fraia @ Cranfield University, 2020
%Marco.Di-Fraia@cranfield.ac.uk

