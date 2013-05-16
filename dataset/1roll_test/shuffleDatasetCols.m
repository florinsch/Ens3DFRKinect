function [ newdata, newlabels ] = shuffleDatasetCols( data, labels )
    if(size(data, 2) ~= size(labels, 2))
        error('Data and labels are not the same size!');
    end
    
    rng('shuffle', 'multFibonacci');
    shuffler = randperm(size(data, 2));
    newdata = data(:, shuffler);
    newlabels = labels(:, shuffler);
end

