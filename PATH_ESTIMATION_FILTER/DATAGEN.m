%% Constants
global Ancillary
Rx = 250; 
Ry = 250;
Rz = 250; 
Rho = 1120; 
Om = 1/3600; 
RAx = [0 0 1];
Ancillary(1) = Rx; Ancillary(2) = Ry; Ancillary(3) = Rz; Ancillary(4) = Rho; Ancillary(5) = Om;
Ancillary(6:8) = RAx; 
%mu = G*Volume*Rho; 
RVe = Om*RAx; %The vector representing the rotation of the target
Gain = Om*3600;
%R_TtoI = @(t) [cos(Om*t), sin(Om*t), 0; -sin(Om*t), cos(Om*t), 0; 0, 0 1];
%To be revised when going into 6 DoF
%R_C2toV = [1, 0, 0; 0 1 0; 0 0 -1];

%% Generate the states along the trajectory using an analytical model
%TrajectoryCDEf(Gain, Rx, Ry, Rz,Rho,GraphicSwitch) has the following
%inputs:
%Gain: a scale factor for the rotational velocity of the target. 
%Given Gain in Rph, omega in rad/s is Gain/3600. Change accordigly.
%Rx, Ry and Rz are the dimensions of the Target Ellipsoid;
%Rho is the density of the target ellipsoid; 
%GraphicSwitch is a parameter which can be either 0 or 1. If 0 graphic is
%suppressed; if 1 graphic is activated

[Xdatagen, tspan] = TrajectoryCDEf(Gain, Rx, Ry, Rz, Rho, 1); %Defined in TCTF

%% Measurement process
%If the 
RealMeasSwitch = 0; 
if RealMeasSwitch == 0
    qqx = 3000; %Artificial variance along the X axis (Position)
    qqy = 4000;  %Artificial variance along the Y axis (Position)
    qqz = 1600;  %Artificial variance along the Z axis (Position)
    Xobs = CorruptX(Xdatagen, qqx, qqy, qqz);
    plot3(Xobs(:,1), Xobs(:,2), Xobs(:,3),'Color', [0,166,86]/255)
elseif RealMeasSwitch == 1
   %X = actualfunction(Xdatagen0);
end

Chck = 1; 
if Chck == 1
    figure
    xx1 = subplot(3,1,1);
    plot(Xobs(:,1) - Xdatagen(:,1))
    xlim(xx1, [1 length(Xdatagen(:,1))])
    xx2 = subplot(3,1,2);
    plot(Xobs(:,2) - Xdatagen(:,2))
    xlim(xx2, [1 length(Xdatagen(:,1))])
    xx3 = subplot(3,1,3);
    plot(Xobs(:,3) - Xdatagen(:,3))
    xlim(xx3, [1 length(Xdatagen(:,1))])
end

Qi = zeros(3,3,length(Xdatagen(:,1)));
R_TtoC_Tensor = zeros(3,3,length(Xdatagen(:,1)));
% TESTING f 
Len = size(Xdatagen); 
Xdatagen = [];
Xdatagen = zeros(Len(1),3); 
Xdatagen(:,2) = linspace(250, 5000, Len(1));
[eulBL] = OrieCentCamBL(Xdatagen); %From Blender to Origin-pointing camera 
Sunn = Om*tspan;
Orb2Table(Xdatagen, Sunn); 