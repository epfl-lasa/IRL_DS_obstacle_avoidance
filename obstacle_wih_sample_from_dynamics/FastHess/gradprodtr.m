% Compute the product of the transpose J matrix and a vector.
function Jg = gradprodtr(A,B,g)

% Get constants.
T = size(g,1);
Dx = size(A,2);

% If we have A and B matrices, perform the multiplication in linear time.
Jg = zeros(T,1,Dx);
Jg(1,1,:) = B(:,:,1)*permute(g(1,1,:),[3 1 2]);
for t=2:T,
    Jg(t,1,:) = A(:,:,t)*permute(Jg(t-1,1,:),[3 1 2]) + B(:,:,t)*permute(g(t,1,:),[3 1 2]);
end;
