function [labels, closest] = getUniqueClusterLabels(data, centroids, ~)
    noObs = size(data, 1);
    noClusters =  size(centroids, 1);
    order = 1:noObs;    

    distances = pdist2(data, centroids, @distEucSq);
    [closest, labels] = min(distances, [] , 2);
    
    if nargin > 2
        return;
    end
    
     % if noObs > noClusters we can't have unique labels
    if noObs > noClusters
        fprintf('Not searching for unique clusters!\n');
        return;
    end
    
    % stops when all labels are unique
    while numel(unique(labels)) ~= noObs
        % sort ascending by closest
        [closest, sortIdx] = sort(closest);
        labels=labels(sortIdx);
        order=order(sortIdx);

        % get first unique items
        [val, pos] = unique(labels, 'first');
        
        stash = zeros(1, noObs); % make empty stash
        stash(pos) = labels(pos); % stash closest unique cluster values
        
        % sort ascending by order
        [order, sortIdx] = sort(order);
        stash = stash(sortIdx); % put stash in original order

        % get leftover idx to search for 2nd min
        leftoverIdx = (stash == 0);

        % get possible clusters (can't be already assigned ones)
        valid = ~ismember(1:noClusters, val);
         % if this is zero, number of patches are larger than noClusters
        if ~any(valid)
            fprintf('Partial unique face region labels...\n');
            break;
        end

        % get distances for leftovers
        leftdist = pdist2(data(leftoverIdx, :), centroids, @distEucSq);
        
        % get new minimum distances
        minVals = min(leftdist(:, valid), [] , 2);
        
        % find index position in original distances matrix
        minIdx = ismember(leftdist, minVals);
        
        % apparently find also sorts so get in original position
        [pos, val] = find(minIdx~=0);
        lbl = val(pos);

        % fill up stash & copy to labels
        stash(leftoverIdx) = lbl; 
        labels = stash;
    end
end