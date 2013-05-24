function [ newdata, newlabels ] = shuffleObservations( data, labels )
    if(size(data, 1) ~= size(labels, 1))
        error('Data and labels must be the same size!');
    end
    
    rng('shuffle', 'multFibonacci');
    shuffler = randperm(size(data, 1));
    newdata = data(shuffler, :);
    newlabels = labels(shuffler, :);
    rng('default');
end