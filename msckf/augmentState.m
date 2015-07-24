function msckfState_aug = augmentState(msckfState, camera, state_k)
% Augments the MSCKF state with a new camera pose    

    C_IG = quatToRotMat(msckfState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * msckfState.imuState.q_IG;
    p_C_G = msckfState.imuState.p_I_G + C_IG' * camera.p_C_I;

    % Build MSCKF covariance matrix
    P = [msckfState.imuCovar, msckfState.imuCamCovar;
        msckfState.imuCamCovar', msckfState.camCovar];
    
    % Camera state Jacobian
    J = calcJ(camera, msckfState.imuState, msckfState.camStates);
    
    N = size(msckfState.camStates,2);
    
    tempMat = [eye(12+6*N); J];
    
    % Augment the MSCKF covariance matrix
    P_aug = tempMat * P * tempMat';
    
    % Break everything into appropriate structs
    msckfState_aug = msckfState;
    msckfState_aug.camStates{N+1}.p_C_G = p_C_G;
    msckfState_aug.camStates{N+1}.q_CG = q_CG;
    msckfState_aug.camStates{N+1}.state_k = state_k;
    msckfState_aug.camStates{N+1}.trackedFeatureIds = [];
    msckfState_aug.imuCovar = P_aug(1:12,1:12);
    msckfState_aug.camCovar = P_aug(13:end,13:end);
    msckfState_aug.imuCamCovar = P_aug(1:12, 13:end);
end