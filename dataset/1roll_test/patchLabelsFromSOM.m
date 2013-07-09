function labels = patchLabelsFromSOM(data, labels, width, height, gui)
    fprintf('\nTraining SOM %ix%i...', width, height); tic;    

    SOM = selforgmap([width height], 100, 3, 'hextop', 'linkdist');
    
    if nargin < 5
        gui = 0;
    end
    
    SOM.trainParam.showWindow = gui; % don't display GUI
    
    SOM = train(SOM, data', 'useParallel', 'yes');
    
    % get face region labels
    labels(:, 4) = num2cell(vec2ind(SOM(data')))';
    
    fprintf('done\n'); toc;
end