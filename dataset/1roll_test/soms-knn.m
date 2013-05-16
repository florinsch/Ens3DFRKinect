clear; clc;
cd /home/flsk/Documents/uni/thesis/facerecord/dataset/2classes_roll/
importData;

%% train SOMs
somN = 3; % number of cells NxN in SOMs

% shuffle dataset
[ESFdata5cm, ESFlbls5cm] = shuffleDatasetCols(ESFdata5cm, ESFlbls5cm);

% create & train SOM
ESFnet5cm = selforgmap([somN, somN]);
ESFnet5cm = train(ESFnet5cm, ESFdata5cm);
% assign cluster labels to each patch
ESFlbls5cm(4,:) = num2cell(vec2ind(ESFnet5cm(ESFdata5cm)));

%% train KNN
% for j = 1:6
    clc; clear lbl clstr sz1 sz2 knn crsval;
    for i = 1:somN * somN
        lbl = []; clstr = []; 

        amir_cluster{i} = amirset(amir_cluster_labels == i, : );
        marco_cluster{i} = marcoset(marco_cluster_labels == i, : );

        sz1 = size(amir_cluster{i}); sz2 = size(marco_cluster{i});
        lbl = [ ones(sz1(1), 1) ; ones(sz2(1), 1) * 2 ];
        clstr = [ amir_cluster{i} ; marco_cluster{i} ];

        knn{i} = ClassificationKNN.fit(clstr, lbl);
        knn{i}.NumNeighbors = 4;

%         crsval = crossval(knn{i}, 'Leaveout', 'on');
%         kloss(i) = kfoldLoss(crsval);
%         softmax(kloss');
%         compet(kloss');

        result{i} = predict(knn{i},clstr)
        
    end
%     mse(j) = mean(kloss);%(kloss ~=0));
% end
% mean(mse)