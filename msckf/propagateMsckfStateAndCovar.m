function msckfState_prop = propagateMsckfStateAndCovar(msckfState, measurements_k, noiseParams)
    
    % Jacobians
    Q_imu = noiseParams.Q_imu;
    F = calcF(msckfState.imuState, measurements_k);
    G = calcG(msckfState.imuState);

    %Propagate State
    msckfState_prop.imuState = propagateImuState(msckfState.imuState, measurements_k);

    % State Transition Matrix
    Phi = eye(size(F,1)) + F * measurements_k.dT; % Leutenegger 2013
    
    % IMU-IMU Covariance
%     msckfState_prop.imuCovar = msckfState.imuCovar + ...
%                                 ( F * msckfState.imuCovar ...
%                                 + msckfState.imuCovar * F' ...
%                                 + G * Q_imu * G' ) ...
%                                         * measurements_k.dT;

    msckfState_prop.imuCovar = Phi * msckfState.imuCovar * Phi' ...
                                + G * Q_imu * G' * measurements_k.dT; % Leutenegger 2013
    
    % Enforce PSD-ness
    msckfState_prop.imuCovar = enforcePSD(msckfState_prop.imuCovar);
                                    
    % Camera-Camera Covariance
    msckfState_prop.camCovar = msckfState.camCovar;
    
    % IMU-Camera Covariance
    msckfState_prop.imuCamCovar = Phi * msckfState.imuCamCovar;
    msckfState_prop.camStates = msckfState.camStates;
end