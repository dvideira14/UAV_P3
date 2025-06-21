A = [zeros(3), eye(3); zeros(3), -3.09*eye(3)];
B = [zeros(3); eye(3)];
Q = eye(6);
R = diag([7.60e-5, 4.70e-5, 108.9]);
K = lqr(A, B, Q, R)
