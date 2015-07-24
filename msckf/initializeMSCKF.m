function [msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(firstImuState, firstMeasurements, camera, state_k, noiseParams)
%INITIALIZEMSCKF Initialize the MSCKF with tracked features and ground
%truth


%Compute the first state
firstImuState.b_g = zeros(3,1);
firstImuState.b_v = zeros(3,1);
msckfState.imuState = firstImuState;
msckfState.imuCovar = noiseParams.initialIMUCovar;
msckfState.camCovar = [];
msckfState.imuCamCovar = [];
msckfState.camStates = {};

msckfState = augmentState(msckfState, camera, state_k);

%Compute all of the relevant feature tracks
featureTracks = {};
trackedFeatureIds = [];

 for featureId = 1:size(firstMeasurements.y,2)
        meas_k = firstMeasurements.y(:, featureId);
        if ~isnan(meas_k(1,1))
                %Track new feature
                track.featureId = featureId;
                track.observations = meas_k;
                featureTracks{end+1} = track;
                trackedFeatureIds(end+1) = featureId;
                %Add observation to current camera
                msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
 end
 
end

