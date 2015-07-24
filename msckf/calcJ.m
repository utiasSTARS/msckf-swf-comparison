function J = calcJ(camera, imuState_k, camStates_k)
% Jacobian of feature observations w.r.t. feature locations

    C_CI = quatToRotMat(camera.q_CI);
    C_IG = quatToRotMat(imuState_k.q_IG);

    J = zeros(6, 12 + 6*size(camStates_k,2));
    J(1:3,1:3) = C_CI;
    J(4:6,1:3) = crossMat(C_IG' * camera.p_C_I);
    J(4:6,10:12) = eye(3);

end