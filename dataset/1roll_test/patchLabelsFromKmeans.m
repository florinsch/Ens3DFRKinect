function [labels, ctroids] = patchLabelsFromKmeans(data, labels, noclusters)
fprintf('\nTraining K-Means, k=%i...', noclusters); tic;

[lbl, ctroids] = kmeans(data, noclusters, 'replicates', 20, ...
    'options', statset('UseParallel', true ), ...
    'emptyaction', 'drop', 'distance', 'sqeuclidean');

labels(:, 4) = num2cell(lbl);

fprintf('done\n'); toc;
end