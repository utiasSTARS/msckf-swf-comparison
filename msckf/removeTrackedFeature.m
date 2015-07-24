function  [updatedMsckfState, featCamStates, camStateIndices] = removeTrackedFeature(msckfState, featureId)
%REMOVETRACKEDFEATURE Remove tracked feature from camStates and extract all
%camera states that include it

    updatedCamStates = msckfState.camStates;
    featCamStates = {};
    camStateIndices = [];
    for c_i = 1:length(updatedCamStates)
        featIdx = find(featureId == updatedCamStates{c_i}.trackedFeatureIds);
        if ~isempty(featIdx)
            updatedCamStates{c_i}.trackedFeatureIds(featIdx) = [];
            camStateIndices(end + 1) = c_i;
            featCamStates{end +1} = updatedCamStates{c_i};
        end
    end
    
    updatedMsckfState = msckfState;
    updatedMsckfState.camStates = updatedCamStates;
end

