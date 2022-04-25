function [x0] = x0_std(GainPer, GainApo)
%This function generates a random orbit from the periapsis and apoapsis
%distance. These are functions of the average radius of the equivalent
%three axis ellipsoid. In particular, given this as Ra, the periapsis is at
%a distance of GainPer*Ra, while the apoapsis at GainApo*Ra. 

%% Constants
G = 6.674E-11; %Gravitational Parameter
global Ancillary 
Rx = Ancillary(1); Ry = Ancillary(2); Rz = Ancillary(3); Rho = Ancillary(4); Om = Ancillary(5);
%Rx, Ry and Rz represent the axes of the 3-axis ellipsoid; Rho the average
%density of the target; Om (omega) the rotational speed of the target,
%assumed to be rotating about [0, 0, 1] in IRF. Should another be of
%interest it can be easily added to the ancillary vector.
RAx = Ancillary(6:8); RVe = Om*RAx;

R = [Rx, Ry, Rz]; Rave = mean(R); 
Rmax = max(R); Rcontrol = sum(R) - Rmax;
if 2*Rcontrol < Rmax
    error('The periapsis is smaller than the largest radius!')
end
rApo = GainApo*Rave;
rPer = GainPer*Rave;

Volume = 4*pi*Rx*Ry*Rz/3;
mu = Rho*G*Volume;

%% Orbital Elements
a = (rPer + rApo)/2;
rC = rPer/rApo;
e = (1 - rC)/(1+rC)

%The following elements are defined as random (from an uniform distribution) angles 
% - Mean Anomaly (M); 
% - Inclination (Inc);
% - Argument of the Periapsis (ArgPeri);
% - Longitude of the Ascending Node (LongAN);

M = deg2rad(180); Inc = deg2rad(25); ArgPeri = deg2rad(0); LongAN = deg2rad(0);
E = KepFun(M,e);
TTA = sqrt((1+e)/(1-e))*tan(E/2);
nu = 2*atan(TTA);
h = sqrt(mu*a*(1-e*e));

r = a*(1-e*cos(E));
p = a*(1-e^2);

%% Initial State
x = r*(cos(LongAN)*cos(ArgPeri+nu) - sin(LongAN)*sin(ArgPeri+nu)*cos(Inc));
y = r*(sin(LongAN)*cos(ArgPeri+nu) + cos(LongAN)*sin(ArgPeri+nu)*cos(Inc));
z = r*(sin(Inc)*sin(ArgPeri+nu));

vx = (x*h*e/r/p)*sin(nu) - h*(cos(LongAN)*sin(ArgPeri+nu) + sin(LongAN)*cos(ArgPeri+nu)*cos(Inc))/r;
vy = (y*h*e/r/p)*sin(nu) - h*(sin(LongAN)*sin(ArgPeri+nu) - cos(LongAN)*cos(ArgPeri+nu)*cos(Inc))/r;
vz = (z*h*e/r/p)*sin(nu) + h*sin(Inc)*cos(ArgPeri+nu)/r;
[x0] = [x,y,z,vx,vy,vz];
%Update initial conditions: from inertial to rotating frame
x0_r = x0(1:3); 
x0_v = x0(4:6); 
x0_v2 = x0_v - cross(RVe, x0_r);
x0(4:6) = x0_v2;
end

%Code written by M. Z. Di Fraia @ Cranfield University, 2020
%Marco.Di-Fraia@cranfield.ac.uk