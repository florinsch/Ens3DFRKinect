function out = smoothFilter( matrix, sigma, window )

items = size(matrix, 1);
len = size(matrix, 2);
out = zeros(items, len);

if nargin < 3
    sigma = 0.01 * len; window = 0.1 * len;
    fprintf('\nNo parameters for sigma and window size');
    fprintf('\nSigma = %05.2f , Window = %05.2f', sigma, window);
end

gauss = linspace(-window/2, window/2, window); % linear
gauss = (1 / (sigma * sqrt(2*pi))) * exp (-gauss.^2 / (2*sigma^2)); % gauss
gauss = gauss / norm(gauss); % normalize

% :)
for i = 1:items
    out(i,:) = conv(matrix(i,:), gauss, 'same');
end

end

