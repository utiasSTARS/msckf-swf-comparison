function imuStates_up = updateStateHistory(imuStates, msckfState, camera, state_k)
% updateStateHistory -- updates IMU state history from current msckfState
%
% INPUTS:   imuStates -- all the IMU states
%           msckfState -- the current msckfState (at timestep state_k)
%           camera -- camera parameters (needed for IMU-to-camera transform)
%           state_k -- the current timestep
%
% OUTPUTS: imuStates_up -- all the IMU states, replaced
%                           by the msckfState versions where available
%

    % Platitude of the day: Everything that hasn't changed must stay the same
    imuStates_up = imuStates;

    % Update the current IMU state
    imuStates_up{state_k}.q_IG = msckfState.imuState.q_IG;
    imuStates_up{state_k}.p_I_G = msckfState.imuState.p_I_G;
    imuStates_up{state_k}.b_g = msckfState.imuState.b_g;
    imuStates_up{state_k}.b_v = msckfState.imuState.b_v;
    imuStates_up{state_k}.covar = msckfState.imuCovar;
    
    % Update IMU states corresponding to active camera poses
    
    % Camera to IMU transformation
    C_CI = quatToRotMat(camera.q_CI);
    q_IC = rotMatToQuat(C_CI');
    p_I_C = - C_CI' * camera.p_C_I;
    
    
    for camIdx = 1:size(msckfState.camStates, 2)             
        q_IG = quatLeftComp(q_IC) * msckfState.camStates{camIdx}.q_CG;
        
        C_IG = quatToRotMat(q_IG);
        
        p_C_G = msckfState.camStates{camIdx}.p_C_G;
        p_I_G = p_C_G + C_IG'*C_CI'*p_I_C;
        
        cam_state_k = msckfState.camStates{camIdx}.state_k;
        
        imuStates_up{cam_state_k}.q_IG = q_IG;
        imuStates_up{cam_state_k}.p_I_G = p_I_G;
    end
end