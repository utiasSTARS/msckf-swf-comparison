function [errorVec] = imuError(kState, kMinus1State, imuMeasurement, deltaT)
%IMUERROR Compute the 6x1 error vector associated with interoceptive measurement

psiVec = imuMeasurement.omega*deltaT;
psiMag = norm(psiVec);
d = imuMeasurement.v*deltaT;

%Compute rotational error (See Lecture8-10)
Phi = cos(psiMag)*eye(3) + (1 - cos(psiMag))*(psiVec/psiMag)*(psiVec/psiMag)' - sin(psiMag)*crossMat(psiVec/psiMag);
eRotMat = kState.C_vi*(Phi*kMinus1State.C_vi)';
eRot = [eRotMat(2,3); eRotMat(3,1); eRotMat(1,2)];

%Compute translational error
eTrans = kState.r_vi_i - (kMinus1State.r_vi_i + kMinus1State.C_vi'*d);

errorVec = [eTrans; eRot];
end

