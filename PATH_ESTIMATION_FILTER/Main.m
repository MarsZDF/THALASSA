%% Main.m - 
warning('off'); %suppresses the warnings for ill-conditioned inverses of covariance matrices
close all
clearvars

% DESCRIPTION:
%   This function act as the main body of a THALASSA process, and the backbone 
%   of its various modules. 
%   THALASSA is defined as a collection of modules to enable development, 
%   prototyping and testing of visual-based autonomous navigation solutions
%   for spacecraft.  
%    
% -------------------------------------------------------------------------

%% Constants
global Ancillary
Rx = 259; Ry = 251; Rz = 234; 
Rho = 1190; %Target Density
Om = 4.29/3600; %Target rotational speed (rps)
RAx = [0 0 1]; %Orientation of the target rotation axis
Ancillary(1) = Rx; Ancillary(2) = Ry; Ancillary(3) = Rz; Ancillary(4) = Rho; Ancillary(5) = Om;
Ancillary(6:8) = RAx; 
G = 6.674E-11;
Volume = 4*pi*Rx*Ry*Rz/3; %Volume of the target ellipsoid [m3]
mu = G*Volume*Rho; 
RVe = Om*RAx; %The vector representing the rotation of the target
Gain = Om*3600;
%R_TtoI = @(t) [cos(Om*t), sin(Om*t), 0; -sin(Om*t), cos(Om*t), 0; 0, 0 1];

%% Generate the states along the trajectory using an analytical model

%TrajectoryCDEf(Gain, Rx, Ry, Rz,Rho,TypeSwitch, DT) has the following
%inputs:
%Gain: a scale factor for the rotational velocity of the target. 
%Given Gain in Rph, omega in rad/s is Gain/3600. Change accordigly.
%Rx, Ry and Rz are the dimensions of the Target Ellipsoid;
%Rho is the density of the target ellipsoid; 
%TypeSwitch is a parameter which can be either 'circ' or 'ecc'. 
%If 'circ' we have a random circular orbit; if 'ecc' an eccentric orbit set
%by setting apoapsis and periapsis distance inside the function. 
%DT is the total simulation time 

%DT =2*pi*sqrt((10*mean([Rx,Ry,Rz]))^3/mu); %Total simulation time m
DT = 8*3600;
[Xanalytics, tspan] = TrajectoryCDEf(Gain, Rx, Ry, Rz, Rho, 'circ', DT); %Defined in TCTF
Xdatagen = Xanalytics(:,1:6); 
Phianalytics = Xanalytics(:,7:42); 

%% Extracts m observation points setting a time condition
%Extract an observation every hH hours (e.g. 0.5 is half an hour) from time
%t1 to time t2 [h]
tspanHours = tspan/3600;
t1 = 0; t2 = 8; hH = 0.75; 
%tHours = tspanHours(tspanHours >= t1 & tspanHours <= t2); 
ObsTimeTheor = t1:hH:t2;

options = odeset('AbsTol',1e-6,'RelTol',1e-9);
x0_A = Xanalytics(1,:); %Concatenate state vector and STM
tSpanSec = ObsTimeTheor*3600;
[~,XdatagenObs] = ode45(@(t,x)CDE_STM2(t,x),tSpanSec,x0_A,options);
plot3(Xdatagen(:,1), Xdatagen(:,2), Xdatagen(:,3),'Color', '#C38F00')
plot3(XdatagenObs(:,1), XdatagenObs(:,2), XdatagenObs(:,3),'h','MarkerSize',10,'MarkerFaceColor','#3D8DAE');

%% Measurement process
RealMeasSwitch = 1; %1 if we are using real measurements, 0 if simulated
[SD1, ~] = size(XdatagenObs);
CG1 = 5; 

Qi = zeros(6,6,SD1); %Covariances associated to the measuring process
xsun = zeros(3, SD1);
xsunCam = zeros(3, SD1);
R_TtoC_Tensor = zeros(3,3,SD1);
R_TtoC_quat = zeros(SD1, 4);
Sunn = -Om*tSpanSec; 
Phi = zeros(6,6,SD1);
% [eulDyn] = OrieCentCamBL2Dyn(XdatagenObs); %From Blender to Dynamics
% [eulBL2] = OrieCentCamBL(XdatagenObs);
[RotMatrix] = OrieCentCamDynRotm(XdatagenObs);
[eulBL3] = OrieCentCamBL2(RotMatrix);
Orb2Table(XdatagenObs, Sunn, RotMatrix); 

for i = 1:SD1
    RADIUS(i) = sqrt(XdatagenObs(i,1)^2 + XdatagenObs(i,2)^2 + XdatagenObs(i,3)^2); 
    xsun(:,i) = [cos(Sunn(i)); sin(Sunn(i)); 0];
    R_TtoC_Tensor(:,:,i) = RotMatrix(:,:,i);
    xsunCam(:,i) = R_TtoC_Tensor(:,:,i)*xsun(:,i);
end

if RealMeasSwitch == 0
    qqx = (CG1/sqrt(3))^2; %Artificial variance along the X axis (Position)
    qqy = (CG1/sqrt(3))^2;  %Artificial variance along the Y axis (Position)
    qqz = (CG1/sqrt(3))^2;  %Artificial varianced along the Z axis (Position)
    qqvx = (0.001/sqrt(3))^2;%Artificial variance along the X axis (Velocity)
    qqvy = (0.001/sqrt(3))^2;%Artificial variance along the Y axis (Velocity)
    qqvz = (0.001/sqrt(3))^2; %Artificial variance along the Z axis (Velocity)
    Qi0 = diag([qqx, qqy, qqz, qqvx, qqvy, qqvz]);
    Xobs = zeros(SD1, 6); 
    phi(:,:,1) = Phi(:,:,1); 
    
    for i = 1:SD1                    
        cq = [];
        Phi(:,:,i) = (reshape(Phianalytics(i,:),6,6));
        %Qi(1:6,1:6,i) = Phi(:,:,i)*Qi0*Phi(:,:,i)';
        Qi(1:6,1:6,i) = Qi0; cQ = diag(Qi(1:6,1:6,i));
        Xobs0(i,1:6) = CorruptX(XdatagenObs(i,1:6), cQ(1), cQ(2), cQ(3), cQ(4), cQ(5), cQ(6));     
        Xobs(i,1:3) = R_TtoC_Tensor(:,:,i)*Xobs0(i,1:3)';
    end
    
elseif RealMeasSwitch == 1
    disp('The program will now be paused to allow the images to be generated. Press a key when done!');
    pause;
    [Xobs, Qi] = Observationf(xsunCam, R_TtoC_Tensor);
end

for i = 1:SD1
    XobsAs(i,:) = R_TtoC_Tensor(:,:,i)'*Xobs(i,1:3)';
end

close all
hold on
plot3(XdatagenObs(:,1), XdatagenObs(:,2), XdatagenObs(:,3),'h','MarkerSize',10,'MarkerFaceColor','#3D8DAE');
plot3(XobsAs(:,1), XobsAs(:,2), XobsAs(:,3),'^','MarkerSize',10,'MarkerFaceColor','#AF2C2C');
plot3(Xdatagen(:,1), Xdatagen(:,2), Xdatagen(:,3),'Color', [195,143,0]/255,'HandleVisibility','off')


Vec3b = xsun';

for i = 1:SD1
    Vec1b(i,:) = XdatagenObs(i,1:3); Vec2b(i,:) = Vec1b(i,:)/norm(Vec1b(i,:));
% 	Reffective(i) = sqrt(XdatagenObs(i,1)^2 + XdatagenObs(i,2)^2 + XdatagenObs(i,3)^2); 
% 	Robserved(i) = sqrt(XobsAs(i,1)^2 + XobsAs(i,2)^2 + XobsAs(i,3)^2); 
	DeltaVector(i,:) = XobsAs(i,1:3) - XdatagenObs(i,1:3); %Difference of vectors
    DeltaRadius(i) = norm(DeltaVector(i,:));
	C(i) = dot(Vec2b(i,:),Vec3b(i,:));
	C2(i) = norm(cross(Vec2b(i,:),Vec3b(i,:)));
	PhaseAngle(i) = rad2deg(atan2(C2(i),C(i)));
    AzO(i) = atan2(XdatagenObs(i,2),XdatagenObs(i,1));
    ElO(i) = atan2(XdatagenObs(i,3),(sqrt(XdatagenObs(i,2)^2 + XdatagenObs(i,1)^2)));
end

%% Filter
x0 = Xdatagen(1,:);
phi0 = eye(6); %initial STM
reshape_phi0 = reshape(phi0,1,36); %reshape STM into row vector
x0_all = [x0,reshape_phi0]; %Concatenate state vector and STM

% Introduce a perturbation in the reference initial state 
Iqqx = 3*(CG1/sqrt(3))^2; %Artificial variance along the X axis (Position)
Iqqy = 3*(CG1/sqrt(3))^2;  %Artificial variance along the Y axis (Position)
Iqqz = 3*(CG1/sqrt(3))^2;  %Artificial variance along the Z axis (Position)
Iqqvx = (0.001/sqrt(3))^2; %Artificial variance along the X axis (Velocity)
Iqqvy = (0.001/sqrt(3))^2;  %Artificial variance along the Y axis (Velocity)
Iqqvz = (0.001/sqrt(3))^2;  %Artificial variance along the Z axis (Velocity)
Q0IOD = diag([Iqqx, Iqqy, Iqqz, Iqqvx, Iqqvy, Iqqvz]);
x0pert = CorruptX(x0_all(1:6), Iqqx, Iqqy, Iqqz, Iqqvx, Iqqvy, Iqqvz); %Artificially perturbed initial state
x0_all(1:6) = x0pert;
options = odeset('AbsTol',1e-6,'RelTol',1e-9);
%[T,Xref] = ode45(@(t,x)CDE_STM(t,x),tSpanSec,x0_all,options);
%[T,Xref] = ode45(@(t,x)CDE_STM(t,x),tspan,x0_all,options);
[~,XrefIn] = ode45(@(t,x)CDE_STM(t,x),tSpanSec,x0_all,options);
[~,XrefInGraph] = ode45(@(t,x)CDE_STM(t,x),tspan,x0_all,options);


%Accommodate the case of unavailable camera estimations 
% if RealMeasSwitch == 1
    [x0new, Delx_Vec, PDelx_Mat, RRi] = NLSCamera(XrefIn, Xobs, tSpanSec, Qi, R_TtoC_Tensor);
% else
%     [x0new, Delx_Vec, PDelx_Mat] = NLSSimulation6x6(XrefIn, Xobs, tSpanSec, Qi, R_TtoC_Tensor);
% end

X0new(1,:) = x0new(end,:);
K5 = 5; %Number of NLS iterations

for kl = 2:K5
    x0_all_update(1:6) = X0new(kl-1,1:6);
    x0_all_update(7:42) = reshape(phi0,1,36); %reshape STM into row vector
    x0new = []; Xref2In = [];
    options = odeset('AbsTol',1e-6,'RelTol',1e-9);
    [T,Xref2In] = ode45(@(t,x)CDE_STM(t,x),tSpanSec,x0_all_update,options);

%      if RealMeasSwitch == 1
          [x0new, Delx_Vec, PDelx_Mat, RRi] = NLSCamera(Xref2In, Xobs, tSpanSec, Qi, R_TtoC_Tensor);
%      else
%          [x0new, Delx_Vec, PDelx_Mat] = NLSSimulation6x6(Xref2In, Xobs, tSpanSec, Qi, R_TtoC_Tensor);
%      end
    X0new(kl,:) = x0new(1:6);
    
end

phi0 = eye(6); %initial STM
reshape_phi0 = reshape(phi0,1,36); %reshape STM into row vector
x0end = [X0new(end,:),reshape_phi0]; %Concatenate state vector and STM
%[~,XrefEndGraph] = ode45(@(t,x)CDE_STM(t,x),tSpanSec,x0end,options);
[~,XrefEndGraph] = ode45(@(t,x)CDE_STM(t,x),tspan,x0end,options);

%% Graphics
close all
[o1, o2] = size(Xdatagen);
for k = 1:o1
R(k) = sqrt(Xdatagen(k,1)^2) + sqrt(Xdatagen(k,2)^2) + sqrt(Xdatagen(k,3)^2);
end
hold on
%plot3(Xref2In(:,1), Xref2In(:,2), Xref2In(:,3),'Color','#3D8DAE')
plot3(XdatagenObs(:,1), XdatagenObs(:,2), XdatagenObs(:,3),'h','MarkerSize',10,'MarkerFaceColor','#3D8DAE');
plot3(XobsAs(:,1), XobsAs(:,2), XobsAs(:,3),'^','MarkerSize',10,'MarkerFaceColor','#AF2C2C');
plot3(XrefInGraph(:,1), XrefInGraph(:,2), XrefInGraph(:,3),'Color',[0,166,86]/255)
plot3(XrefEndGraph(:,1), XrefEndGraph(:,2), XrefEndGraph(:,3),'Color','#3D8DAE')
%plot3(XrefIn(:,1), XrefIn(:,2), XrefIn(:,3),'Color', [0,166,86]/255)
plot3(Xdatagen(:,1), Xdatagen(:,2), Xdatagen(:,3),'Color', [255,98,36]/255)
legend('{X^{Obs}}_{Analytics}', 'X^{Obs}', '{X^{ref}}_{Initial}', '{X^{ref}}_{Estimated}', 'X_{Analytics}', 'FontSize',18);
legend

% if RealMeasSwitch == 1
%     Analytics2
% else
%     Analytics2Cam
% end

%% AUTHORS
% Version 1.0 written by Marco Zaccaria Di Fraia in 2020
% Marco.Di-Fraia@cranfield.ac.uk