function [] = runAllClusterTypes(wriData, wriLbls, allKs, somW, somH, fName)
    noClusters = somW * somH;
    noAllKs = numel(allKs);
    
    % shuffle dataset
    [wriData, wriLbls] = shuffleObservations(wriData, wriLbls);
    
    % SOMs
    wriLbls = patchLabelsFromSOM(wriData, wriLbls, somW, somH);
    fid = fopen(fName,'a'); fprintf(fid,'SOM %i x %i\n', somW, somH); fclose(fid);
    for i = 1:noAllKs
        [fEns, fOne] = LOOCV(wriData, wriLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
    end

    %  i-Means
    wriLbls = patchLabelsFromimeans(wriData, wriLbls, noClusters);
    fid = fopen(fName,'a'); fprintf(fid,'KMEANS K = %i\n', noClusters); fclose(fid);
    for i = 1:noAllKs
        [fEns, fOne] = LOOCV(wriData, wriLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
    end
    
    % patch labels from uniform random distribution of cluster labels
    wriLbls(:, 4) = num2cell(unidrnd(noClusters, noObservations, 1));
    fid = fopen(fName,'a'); fprintf(fid,'RANDOM'); fclose(fid);
    for i = 1:noAllKs
        [fEns, fOne] = LOOCV(wriData, wriLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
    end
    
    % get class labels from patch grid ordering
    [wriData, wriLbls] = patchLabelsFromGrid(wriData, wriLbls);
    fid = fopen(fName,'a'); fprintf(fid,'GRID'); fclose(fid);
    for i = 1:noAllKs
        [fEns, fOne] = LOOCV(wriData, wriLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
    end