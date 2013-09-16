function D = distEucSq( X, Y )

%if( ~isa(X,'double') || ~isa(Y,'double'))
 % error( 'Inputs must be of type double'); end;
m = size(X,1); n = size(Y,1);
%Yt = Y';
XX = sum(X.*X,2);
YY = sum(Y'.*Y',1);
D = XX(:,ones(1,n)) + YY(ones(1,m),:) - 2*X*Y';