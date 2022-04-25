function [X1, tspan] = TrajectoryCDEf(Gain, Rx, Ry, Rz,Rho,TypeSwitch, DT)
clearvars -EXCEPT counter Counter Gain Rx Ry Rz Rho TypeSwitch DT; 
%close all;  
clc; 
tic 
%% User inputs
% TestCase = 10; % Select a number between 1 and 10 (10 is still experimental)
% Angle9 = 46; %Rotation about the Y axis for case 8 and X axis for case 9 [degree]
ACCEL = 0; %1 to obtain information on the acceleration, 0 otherwise
Graphic = 1; %1 to display the trajectory; 0 otherwise

%% Constants
G = 6.674E-11; %Gravitational Constant 
%Gain = 0.25; %Number of rotations per hour of the target
%Gain2 = 10; %Used in rotational trajectories test cases to define the initial position
% Position = Gain2*Radius, where Radius is one of the target's radii.

% Target Parameters 
% Rx = 250; %The X semi-axis of the ellipsoid [m]
% Ry = 250; %The Y semi-axis of the ellipsoid [m]
% Rz = 250; %The Z semi-axis of the ellipsoid [m]
Volume = 4*pi*Rx*Ry*Rz/3; %Volume of the target ellipsoid [m3]
%Rho = 1120; %The average density of the target [kg/m3] 
mu = G*Volume*Rho; %Standard gravitational parameter G*Mass, where mass is retrieved as the product of V and density
Om = Gain/3600; %The uniform rotational speed (scalar, omega) of the target in [rad/s]

% Global Parameters - used to propagate the above defined constant
% throughout the code
global Ancillary
Ancillary(1) = Rx; Ancillary(2) = Ry; Ancillary(3) = Rz; 
Ancillary(4) = Rho; Ancillary(5) = Om;
%RAx represent the orientation of the rotation axis 
RAx0_x = 0; RAx0_y = 0; RAx0_z = 1; RAx0 = [RAx0_x, RAx0_y, RAx0_z];
RAx = RAx0/norm(RAx0); Ancillary(6:8) = RAx; RVe = Om*RAx;

%% Simulation-case-specific Parameters 
n = 2500; % Number of simulation iteration 
%DT = 5.5e+05; %(s) Simulation total time 
%dt = DT/n; %(s) timestep 
tspan0 = [0 DT];
tspan = linspace(0,DT,n); 
options = odeset('AbsTol',1e-6,'RelTol',1e-9);

%Initial Conditions in the Inertial frame
%The built-in test cases describe circular orbits
switch TypeSwitch
    case 'circ'
        [x0] = x0_CircularRandom;   %Experimental Feature
    case 'circ2'
        [x0] = x0_CircularRandom2;   %Experimental Feature
    case 'ecc'
        [x0] = x0_EccentricRandom(2, 10);
    case 'std'
        [x0] = x0_std(5, 15);%Standard testing case
       
end

%Functions to include the STM
phi0 = eye(6); %initial STM
reshape_phi0 = reshape(phi0,1,36); %reshape STM into row vector
x0_all = [x0,reshape_phi0]; %Concatenate state vector and STM

%% Solver
%Retrieve Solutions and time 
%[T,X] = ode45(@(t,x)CDEEoM(t,x),tspan,x0,options);
[T,X1] = ode45(@(t,x)CDE_STM2(t,x),tspan,x0_all,options);
X = X1(:,1:6);

%% Check for invalid solutions
%Ancillary vector to control that no solution point goes within the
%ellipsoid
ancX = zeros([n 1]);
for i = 1:length(X)
ancX(i) = (X(i,1)/Rx)^2 + (X(i,2)/Ry)^2 + (X(i,3)/Rz)^2;
end
lim = find(ancX < 0.9999, 1, 'first') - 1;

if isempty(lim)
    XValid = X; 
    TotalTime = -(T(1) - T(length(X)))/3600;
elseif find(ancX < 0.9999, 1, 'first') == 1
    XValid = X;
    TotalTime = -(T(1) - T(length(X)))/3600;
else
    XValid = X(1:lim, :);
    TotalTime = -(T(1) - T(lim))/3600;
end

OldX = X; 
X = XValid; 

%% Graphic
if Graphic == 1
%Randomize plot colours for multiple consecutive plots
Gra = rand;
if Gra < 0.2 
    plot3(X(:,1), X(:,2), X(:,3),'Color', [206,159,183]/255,'HandleVisibility','off')
elseif Gra >= 0.2 && Gra < 0.4
    plot3(X(:,1), X(:,2), X(:,3),'Color', [61,141,174]/255,'HandleVisibility','off')
elseif Gra >= 0.4 && Gra < 0.6
    plot3(X(:,1), X(:,2), X(:,3),'Color', [0,54,61]/255,'HandleVisibility','off')
elseif Gra >= 0.6 && Gra < 0.8
    plot3(X(:,1), X(:,2), X(:,3),'Color', [255,98,36]/255,'HandleVisibility','off')
elseif Gra >= 0.8 && Gra <= 1
    plot3(X(:,1), X(:,2), X(:,3),'Color', [195,143,0]/255,'HandleVisibility','off')
end
[alfa, beta] = size(OldX);
[gamma, theta] = size(X);

hold on

X0 = X(1,1);
Y0 = X(1,2);
Z0 = X(1,3);
[x,y,z] = sphere;
handle = surfl(x*25 + x0(1), y*25 + x0(2), z*25 + x0(3));
set(handle, 'FaceAlpha', 0.9);
handle.Annotation.LegendInformation.IconDisplayStyle = 'off';
handle.FaceColor = [0,54,61]/255;
%shading interp
%https://uk.mathworks.com/matlabcentral/answers/59536-how-to-create-a-transparent-sphere


[ex,ey,ez] = ellipsoid(0,0,0,Rx,Ry,Rz,40);
handlellipse = surfl(ex, ey, ez); 
set(handlellipse, 'FaceAlpha', 0.5)
handlellipse.Annotation.LegendInformation.IconDisplayStyle = 'off';
handlellipse.FaceColor = [175,44,44]/255;

axis equal

end 


%% Control functions

%Compute and plot the de-rotated trajectory
R_ItoT = @(t) [cos(Om*t), sin(Om*t), 0; -sin(Om*t), cos(Om*t), 0; 0, 0 1];
[o1 o2] = size(OldX);
for k = 1:o1
    Xderot(k,1:3) = inv(R_ItoT(T(k)))*XValid(k,1:3)';
    R1(k) = sqrt(X(k,1)^2 + X(k,2)^2 + X(k,3)^2);
    R2(k) = sqrt(Xderot(k,1)^2 + Xderot(k,2)^2 + Xderot(k,3)^2);
end

hold on
plot3(Xderot(:,1), Xderot(:,2), Xderot(:,3), 'Color', [0,54,61]/255,'HandleVisibility','off')
%axis equal
%Plot distance from center
%figure
%plot(R1-R2, 'Color', [0,49,83]/255);
%plot(R2, 'Color', [53, 133, 151]/255);

ttt = toc; %disp(ttt)
%% Time and Acceleration
if  ACCEL == 1
    [a] = CDE_STM_acc(XValid); 
end

%Total time observed in the simulation: 
TotTim = ['Total Time Observed in the Simulation: ', num2str(TotalTime), ' hours, ', num2str(TotalTime/24), ' days'];
disp(TotTim)

tic 


%Code written by M. Z. Di Fraia @ Cranfield University, 2020
%Marco.Di-Fraia@cranfield.ac.uk