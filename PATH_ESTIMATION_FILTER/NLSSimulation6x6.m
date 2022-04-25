function [x0new, Delx_Vec, PDelx_Mat] = NLSSimulation6x6(Xref, Xobs, T, Qi, RTensor)
%Non-linear Least Squares
%Inputs:
%Xrefis the matrix of reference states
%Xobs the matrix of state observations
%T is the matrix containing the timesteps
%Qi is the matrix containing the covariance of the 

%% Matrices allocation:
% !!! Given a matrix S, Si represents that matrix for the timestep i. 
%Hi is the derivative w.r.t. the state of the observation relation
Hi = zeros(6,6, length(T)); 
%Ti represents the observation matrix Hi*Phi
Ti = zeros(6,6, length(T));
%State Transition Matrix
PhiT = zeros(6,6, length(T));
%Residuals
RRi = zeros(length(T),6);
%Running sum of 
SUM1_Mat = zeros(6,6,length(T));
%Running sum of 
SUM2_Vec = zeros(6,length(T));
%Tensor containing the inverse of the covariance matrix
QiInv = zeros(6,6,length(T));
%The tensor containing the covariances is considered given

I6 = eye(6,6);

%This process is the one explained in page 74 of Modern Orbit Determination
for tt = 1:length(T)
    PhiT(:,:,tt) = reshape(Xref(tt,7:end),6,6);
    QiInv(:,:,tt) = inv(Qi(:,:,tt)); 
    RRi(tt,:) = Xobs(tt,1:6)' - Xref(tt,1:6)'; %Obtain the residuals
    %RRi(tt,:) = Xobs(tt,1:6)' - I6*Xref(tt,1:6)'; %Obtain the residuals
    %Hi(1:6,1:6, tt) = I6;
    %Ti(:,:,tt) = Hi(:,:, tt)*PhiT(:,:,tt);
    Ti(:,:,tt) = PhiT(:,:,tt);
    if tt > 1
        SUM1_Mat(:,:, tt) = SUM1_Mat(:,:,tt-1) + (transpose(Ti(:,:,tt)))*QiInv(:,:,tt)*Ti(:,:,tt);
        SUM2_Vec(:,tt) = SUM2_Vec(:, tt-1) + (transpose(Ti(:,:,tt)))*QiInv(:,:,tt)*RRi(tt,:)';
    end 
end
RRi;
PDelx_Mat = inv(SUM1_Mat(:,:,end));
Delx_Vec = PDelx_Mat*SUM2_Vec(:,end); %#ok<MINV>
x0new = Xref(1,1:6);
x0new(1:6) = x0new(1:6) + Delx_Vec';
end

%Let RTensor be the tensor containing all the rotation matrices 
%between TRF and CRF from t=0 to t=TOF. Then at a specific time 
%the measurement matrix is a 3x6 matrix containing a 3x3 R on its left side
%and a 3x3 matrix of zeros on its right side. Since we'll be assuming the
%camera to be oriented towards the center of the target, and directed from
%this center to this sensor, this will be obtainable from the state vector
%itself.