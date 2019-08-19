% Compute the product of the J matrix and a vector.
function Jg = gradprod(A,B,g)

% Get constants.
T = size(g,1);
Du = size(B,2);
Dx = size(A,2);

% If we have A and B matrices, perform the multiplication in linear time.
Jg = zeros(T,1,Du);
prsum = zeros(Dx,1);
for t=T:-1:1
    prsum = permute(g(t,1,:),[3 1 2]) + prsum;
    Jg(t,1,:) = B(:,:,t)'*prsum;
    prsum = A(:,:,t)'*prsum;
end
