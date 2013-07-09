function [  observations, noObservations, classes, noClasses, ...
            noPatches, avgPatchesPerObs] = getDatasetStats(labels)
        
    observations = unique(labels(:, 1));
    noObservations = numel(observations);
    fprintf('Number of observations (camera frames): %d\n', noObservations);

    noPatches = numel(labels(:, 1));
    fprintf('Number of patches (total): %d\n', noPatches);

    classes = unique(labels(:, 3));
    noClasses = numel(classes);
    fprintf('Number of classes (people): %d\n', noClasses);

    % average number of patches per frame (camera observation)
    avgPatchesPerObs = noPatches/noObservations;
    fprintf('Average number of patches per frame: %f\n', avgPatchesPerObs);
end