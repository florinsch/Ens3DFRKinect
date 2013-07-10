function [F1score, precision, recall, specificity, accuracy, TP, FP, FN, TN ] = statsPerClass(confMat)
    noClsz = size(confMat, 1);
    noObs = sum(confMat(:));
    
    TP = zeros(1, noClsz);
    FP = zeros(1, noClsz);
    FN = zeros(1, noClsz);
    TN = zeros(1, noClsz);

    accuracy    = zeros(1, noClsz);
    precision   = zeros(1, noClsz);
    recall      = zeros(1, noClsz);
    specificity = zeros(1, noClsz);
    F1score     = zeros(1, noClsz);
    
    for i = 1:noClsz
        sumH = sum(confMat(i, :)) + eps; % avoid division by 0
        sumV = sum(confMat(:, i)) + eps;
        
        TP(i) = confMat(i, i);
        FP(i) = sumV - TP(i);
        FN(i) = sumH - TP(i);
        TN(i) = noObs - sumH - sumV + TP(i);
        
        accuracy(i) = (TP(i) + TN(i)) / (TP(i) + TN(i) + FP(i) + FN(i));
        % Precision(class) = TP(class) / ( TP(class) + FP(class) )
        precision(i) = TP(i) / sumV ;
        % Recall(class) == Sensitivity(class) == TruePositiveRate(class)
        % = TP(class) / ( TP(class) + FN(class) )
        recall(i) = TP(i) / sumH;
        % Specificity == TrueNegativeRate(class)
        % = TN(class) / ( TN(class) + FP(class) )
        specificity(i) = TN(i) / ( TN(i) + FP(i) );
        % F1-measure = 2 * ( ( P * R ) / (P + R) )
        F1score(i) = 2*(precision(i)*recall(i)) / (precision(i)+recall(i));

        % False positive rate = 1 − specificity = FP / (FP + TN)
        % False negative rate = 1 − sensitivity = FN / (TP + FN)
 
        % per(i,1) false negative rate
        %   = (false negatives)/(all output negatives)
        % per(i,2) false positive rate
        %   = (false positives)/(all output positives)
        % per(i,3) true positive rate
        %   = (true positives)/(all output positives)
        % per(i,4) true negative rate
        %   = (true negatives)/(all output negatives)
    end
%     accuracy(~isfinite(accuracy)) = 0;
%     precision(~isfinite(precision)) = 0;
%     recall(~isfinite(recall)) = 0;
%     specificity(~isfinite(specificity)) = 0;
    % Model Accuracy
    % modelAcc = sum(diag(confMat))/noObs;
end