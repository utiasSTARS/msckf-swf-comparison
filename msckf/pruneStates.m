function [ prunedMsckfState, deletedCamStates ] = pruneStates( msckfState )
%PRUNESTATES Prunes any states that have no tracked features and updates
%covariances
    
    prunedMsckfState.imuState = msckfState.imuState;
    prunedMsckfState.imuCovar = msckfState.imuCovar;
    
    %Find all camStates with no tracked landmarks    
    deleteIdx = [];
    for c_i = 1:length(msckfState.camStates)
        if isempty(msckfState.camStates{c_i}.trackedFeatureIds)
            deleteIdx(end+1) = c_i;
        end
    end
    
    %Prune the damn states!
    
    deletedCamStates = msckfState.camStates(deleteIdx);
    prunedMsckfState.camStates = removeCells(msckfState.camStates, deleteIdx);
    
    statesIdx = 1:size(msckfState.camCovar,1);
    keepCovarMask = true(1, numel(statesIdx));
    for dIdx = deleteIdx
        keepCovarMask(6*dIdx - 5:6*dIdx) = false(6,1);
    end
    
    keepCovarIdx = statesIdx(keepCovarMask);
    deleteCovarIdx = statesIdx(~keepCovarMask);
    
    prunedMsckfState.camCovar = msckfState.camCovar(keepCovarIdx, keepCovarIdx);
    %Keep rows, prune columns of upper right covariance matrix
    prunedMsckfState.imuCamCovar = msckfState.imuCamCovar(:, keepCovarIdx);
    
    deletedCamCovar = msckfState.camCovar(deleteCovarIdx, deleteCovarIdx);
    deletedCamSigma = sqrt(diag(deletedCamCovar));
    
    % Grab the variances of the deleted states for plotting
    for c_i = 1:size(deletedCamStates, 2)
        deletedCamStates{c_i}.sigma = deletedCamSigma(6*c_i - 5 : 6*c_i);
    end
end

