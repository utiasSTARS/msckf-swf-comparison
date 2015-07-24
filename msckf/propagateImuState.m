function imuState_prop = propagateImuState(imuState_k, measurements_k)
% prop == propagated to k+1

    C_IG = quatToRotMat(imuState_k.q_IG);
    
    % Rotation state
    psi = (measurements_k.omega - imuState_k.b_g) * measurements_k.dT;
    imuState_prop.q_IG = imuState_k.q_IG + 0.5 * omegaMat(psi) * imuState_k.q_IG;
%     diffRot = axisAngleToRotMat(psi);
%     C_IG_prop = diffRot * C_IG;
%     imuState_prop.q_IG = rotMatToQuat(C_IG_prop);
    
    %Unit length quaternion
    imuState_prop.q_IG = imuState_prop.q_IG/norm(imuState_prop.q_IG);
    
    % Bias states
    imuState_prop.b_g = imuState_k.b_g;
    imuState_prop.b_v = imuState_k.b_v;
    
    % Translation state
    d = (measurements_k.v - imuState_k.b_v) * measurements_k.dT;
    imuState_prop.p_I_G = C_IG' * d + imuState_k.p_I_G;
    
end