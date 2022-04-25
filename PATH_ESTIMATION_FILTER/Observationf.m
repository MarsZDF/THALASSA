function [r_pos, Pr_mat] = Observationf(xsunCam, R_TtoC_Tensor)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Constants
global Ancillary
Rx = Ancillary(1); Ry = Ancillary(2); Rz = Ancillary(3); Rho = Ancillary(4); Om = Ancillary(5);
RAx = Ancillary(6:8); RVe = Om*RAx;
%Rx = Rx*2; Ry = Ry*2; Rz = Rz*2; %IT HAS TO BE SEMI AXES!
%Number of points sampled on the ellipse
M = 600; 
%Intrinsic Matrix
fo_length = 9.6/1000; %Focal Length, 55 mm in m 
PixS =  2.6764e-05; %Pixel size : 5.30 micrometers
Skew = 0; mx = 1/PixS; my = 1/PixS;
%Sensor size : 1280x720 pixels CxR %Sensor size : 6.78x5.43 mm (H*PixSize/100, W*PixSize/100) 
px0 = 640; py0 = 360;
IntMat = [fo_length*mx, Skew, px0; 0, fo_length*my, py0; 0 0 1];
%IFOV = ; %IFOV in mrad

%Shape Matrix
Ap = diag([1/(Rx^2), 1/(Ry^2), 1/(Rz^2)]);

%% Read all the images 
cd 'C:\Users\ingma\OneDrive\Desktop\THALASSA\OutputBlender';
%cd 'C:\Users\Marco\Desktop\TestEnv\TestData\S0_Compare\R5S0\BData\B3'; 
%cd C:\Users\Marco\Desktop\TestEnv\TestData\S0_Compare\R5S0\EData\E3;
%cd C:\Users\Marco\Desktop\TestEnv\TestData\S0\R5S0\TestData5;
Images = dir('*.png');      
nIm = length(Images);    % Number of files found

% Memory Allocations
r_pos = zeros(nIm,3);
Pr_mat = zeros(3,3,nIm);

for ii=1:nIm
   ImageSet{ii} = imread(Images(ii).name);
end
 [~, reindex] = sort( str2double( regexp( {Images.name}, '\d+', 'match', 'once' )));
 ImageSet = ImageSet(reindex);

%% Detect the ellipse and construct the matrix of points in the image plane
 %for jki = 1:nIm 
 for jki = 1:nIm 
     jki
     s3_mat = [];
     jki;
     xsunCam2(:,jki) = [xsunCam(1,jki)/xsunCam(3,jki), xsunCam(2,jki)/xsunCam(3,jki), 1];
     A(:,:,jki) = R_TtoC_Tensor(:,:,jki)*Ap*((R_TtoC_Tensor(:,:,jki))^(-1));
     Sun_dir_imageAngle(jki) = rad2deg(atan2(xsunCam2(1,jki),xsunCam2(2,jki)));
    [a, b, c, d, e, f, Points] = EstEll(ImageSet{jki},Sun_dir_imageAngle(jki));
    %Convert to parametric form and return M points: 
    [s3_mat] = SamplingEllipse(a, b, c, d, e, f, M);
    s3(:,:,jki) = s3_mat; 
    
    for i = 1:length(s3_mat)
        s3_mat(i,3) = 1; 
        %s3_mat(i,:) = (inv(IntMat)*s3_mat(i,:)');
        %s3_mat(i,:) = ((IntMat)*s3_mat(i,:)');
        s3_mat(i,:) = (IntMat\s3_mat(i,:)');
    end

    %% Determine the position of the camera through the Cholesky factorization process
    [r_pos0, Pr_mat0] = CholFRN(A(:,:,jki), s3_mat);
    r_pos(jki,:) = r_pos0;
    Pr_mat(:,:,jki) = Pr_mat0;
 end

