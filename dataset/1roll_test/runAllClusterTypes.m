function [] = runAllClusterTypes(wrkData, wrkLbls, allKs, somW, somH, fName)
    
    % hack to generate unique frame names (concat frame + class in frame)
%     wrkLbls(:, 1) = strcat(wrkLbls(:, 1), wrkLbls(:,3));
%     noPatches = numel(wrkLbls(:, 1));
    
    noClusters = somW * somH;
    noAllKs = numel(allKs);
    
    % shuffle dataset
    % [wrkData, wrkLbls] = shuffleObservations(wrkData, wrkLbls);
    % this changes the results (observable for OneKNN)
    
    % SOMs
    wrkLbls = patchLabelsFromSOM(wrkData, wrkLbls, somW, somH);
    fid = fopen(fName,'a'); fprintf(fid,'SOM %i x %i\n', somW, somH); fclose(fid);
    for i = 1:noAllKs
        [fMacro, fMicro] = LOOCV(wrkData, wrkLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fMacro, fMicro], '-append');
    end

    %  K-Means
    wrkLbls = patchLabelsFromKmeans(wrkData, wrkLbls, noClusters);
    fid = fopen(fName,'a'); fprintf(fid,'KMEANS K = %i\n', noClusters); fclose(fid);
    for i = 1:noAllKs
        [fMacro, fMicro] = LOOCV(wrkData, wrkLbls, allKs(i));
        dlmwrite(fName, [allKs(i), fMacro, fMicro], '-append');
    end
    
    % one KNN
    fid = fopen(fName,'a'); fprintf(fid,'One KNN\n'); fclose(fid);
    for i = 1:noAllKs
        [fMacro, fMicro] = LOOCV(wrkData, wrkLbls, allKs(i), 'single');
        dlmwrite(fName, [allKs(i), fMacro, fMicro], '-append');
    end
    
    % patch labels from uniform random distribution of cluster labels
%     wrkLbls(:, 4) = num2cell(unidrnd(noClusters, noPatches, 1));
%     fid = fopen(fName,'a'); fprintf(fid,'RANDOM\n'); fclose(fid);
%     for i = 1:noAllKs
%         [fEns, fOne] = LOOCV(wrkData, wrkLbls, allKs(i));
%         dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
%     end
    
    % get class labels from patch grid ordering
%     [wrkData, wrkLbls] = patchLabelsFromGrid(wrkData, wrkLbls);
%     fid = fopen(fName,'a'); fprintf(fid,'GRID\n'); fclose(fid);
%     for i = 1:noAllKs
%         [fEns, fOne] = LOOCV(wrkData, wrkLbls, allKs(i));
%         dlmwrite(fName, [allKs(i), fEns, fOne], '-append');
%     end