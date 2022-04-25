M = 600; %Number of points sampled on the ellipse
%% Intrinsic Matrix
fo_length = 0.015; %Focal Length, 15 mm in m 
PixS = 5.3e-6; %Pixel size : 5.30 micrometers
Skew = 0; mx = 1/PixS; my = 1/PixS;
%Sensor size : 1280x1024 pixels CxR
%Sensor size : 6.78x5.43 mm (H*PixSize/100, W*PixSize/100) 
%Skew = 0; 
px0 = 512; py0 = 640;
IntMat = [fo_length*mx, Skew, px0; 0, fo_length*my, py0; 0 0 1];
%% Test Cases
%Attitude values as defined in UE4
RollCamera = 0; PitchCamera = 0; YawCamera = 0;
if TestCase == 1
Rx = 5; Ry = 5; Rz = 5; 
RollPlanet = 0; PitchPlanet = 0; YawPlanet = 0;
I = imread('Test1.png');
%UE: X = -4000.0; Y = 0; Z = 0; Real World: X = -40 m; Y = 0; Z = 0;
elseif TestCase == 2 
Rx = 5; Ry = 10; Rz = 5; 
RollPlanet = 0; PitchPlanet = 0; YawPlanet = 0;
I = imread('Test2.png');
%UE: X = -5000.0; Y = 0; Z = 0; Real World: X = -50 m; Y = 0; Z = 0;
elseif TestCase == 3 
Rx = 8; Ry = 12; Rz = 6; 
RollPlanet = 0; PitchPlanet = 0; YawPlanet = 0;
I = imread('Test3.png');
%UE: X = -6500.0; Y = 0; Z = 0; Real World: X = -65 m; Y = 0; Z = 0;
elseif TestCase == 4
Rx = 8; Ry = 12; Rz = 6; 
I = imread('Test4.png');
RollPlanet = 0; PitchPlanet = 0; YawPlanet = pi/6;
%UE: X = -6500.0; Y = 0; Z = 0; Real World: X = -65 m; Y = 0; Z = 0;
elseif TestCase == 5
Rx = 8; Ry = 12; Rz = 6; 
I = imread('Test5.png');
RollPlanet = pi/4; PitchPlanet = 0; YawPlanet = pi/6;
%UE: X = -6500.0; Y = 0; Z = 0; Real World: X = -65 m; Y = 0; Z = 0;
elseif TestCase == 6
Rx = 8; Ry = 12; Rz = 6; 
I = imread('Test6.png');
RollPlanet = pi/4; PitchPlanet = 0; YawPlanet = pi/6;
%UE: X = -8000.0; Y = 1000; Z = -600; Real World: X = -80 m; Y = 10; Z = -6;
elseif TestCase == 7
Rx = 8; Ry = 12; Rz = 6; 
I = imread('Test7.png');
RollPlanet = pi/4; PitchPlanet = 0; YawPlanet = pi/6;
%UE: X = -200000.0; Y = 4000.0; Z = 25000.0; Real World: X = -2000 m; Y = 40; Z = 250;
elseif TestCase == 8
Rx = 8; Ry = 12; Rz = 6; 
I = imread('Test8.png');
RollPlanet = pi/4; PitchPlanet = 0; YawPlanet = pi/6;
%UE: X = -100000.0; Y = 7500.0; Z = 0; Real World: X = -1000 m; Y = 75; Z = 0;
elseif TestCase == 9
Rx = 12; Ry = 4; Rz = 8; 
I = imread('Test9.png');
RollPlanet = 0; PitchPlanet = deg2rad(75); YawPlanet = 0;
%UE: X = -12000.0; Y = -2000.0; Z = 1000.0; Real World: X = -120 m; Y = -20; Z = 10;
elseif TestCase == 10
    
end
%% Define all the rotation matrices and rotate the shape matrix 
%Rotation from UE spawned camera to Z-as-boresight camera in RH Coord Frame
Ax = 0; Ay = pi/2; Az = pi/2;
%Rotation matrices
T_ItoUCam = eul2rotm(UE2MAT([RollCamera, PitchCamera, YawCamera],'a'), 'XYZ');
T_UCamtoCam = inv(eul2rotm([Ax, Ay, Az], 'XYZ'));
T_ItoC = T_UCamtoCam*T_ItoUCam; 
T_ItoP = eul2rotm(UE2MAT([RollPlanet, PitchPlanet, YawPlanet],'a'), 'XYZ');
T_PtoC = T_ItoC*inv(T_ItoP); 
Rx = 0.5*Rx; Ry = 0.5*Ry; Rz = 0.5*Rz;
Ap = diag([1/(Rx^2), 1/(Ry^2), 1/(Rz^2)]);
A = T_PtoC*Ap*(T_PtoC)^(-1); 
%% Detect the ellipse and construct the matrix of points in the image plane
[a, b, c, d, e, f, Points] = EstEll(I);
%Convert to parametric form and return M points: 
[s3_mat] = SamplingEllipse(a, b, c, d, e, f, M);

for i = 1:length(s3_mat)
    s3_mat(i,3) = 1; 
    s3_mat(i,:) = (inv(IntMat)*s3_mat(i,:)');
end
%% Determine the position of the camera through the Cholesky factorization process
[r_pos] = CholFRN(A, s3_mat);
%R_UE = T_UCamtoCam\r_pos'

%% Explanation of test cases: 
%1 Perfect Sphere
%2 Spheroid
%3 Ellipsoid
%4 Rotated Ellipsoid - 1 rot
%5 Rotated Ellipsoid - 2 rot
%6 Off-axis Rotated Ellipsoid
%7 Far Range (the failure appears to be due to an overestimation of
%the ellipse) 