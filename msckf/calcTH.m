function [T_H, Q_1] = calcTH(H_o)
%CALCTH Calculates T_H matrix according to Mourikis 2007

[Q,R] = qr(H_o);

%Find all zero rows of R
isZeroRow = all(R==0, 2);

%Extract relevant matrices
T_H = R(~isZeroRow, :);
Q_1 = Q(:, ~isZeroRow);

end

