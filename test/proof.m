%% define an orthogonal matrix via Cayley's formula
syms x y z real
Q = [1+x^2-y^2-z^2, 2*x*y-2*z, 2*y+2*x*z; 2*x*y+2*z, 1-x^2+y^2-z^2, 2*y*z-2*x; 2*x*z-2*y, 2*x+2*y*z, 1-x^2-y^2+z^2];

%% check orthogonality
% The results are s^2*diag([1, 1, 1]), where s = x^2+y^2+z^2+1.
simplify(Q*Q')

%% construct a 3*3 random matrix N in Eq.(15)
imax = 10;
c = cell(3, 1);
for i = 1:3
    c{i} = cross(randi(imax, [3,1]), Q*randi(imax, [3,1]));
end
C1 = [c{1}, c{2}, c{3}];
eq1 = det(C1);

%% check the determinant has factor x^2+y^2+z^2+1
factor(eq1)

%% construct a 4*4 random matrix N in Eq.(14)
c = cell(4, 1);
mm = [x^2; y^2; z^2; x*y; x*z; y*z; x; y; z; 1];
for i = 1:4
    tmp = randi(imax, [1, 10])*mm;
    c{i} = [cross(randi(imax, [3,1]), Q*randi(imax, [3,1])); tmp];
end
C2 = [c{1}, c{2}, c{3}, c{4}];
eq2 = det(C2);

%% check the determinant has factor x^2+y^2+z^2+1
factor(eq2)
