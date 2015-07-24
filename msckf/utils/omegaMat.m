function bigOmega = omegaMat(omega)
%
% Computes the Omega matrix of a 3x1 vector, omega
%
    if(size(omega,1) ~= 3 || size(omega,2) ~= 1)
        error('Input vector must be 3x1');
    end
    
    bigOmega = [ -crossMat(omega),  omega;
                 -omega',            0 ];
end