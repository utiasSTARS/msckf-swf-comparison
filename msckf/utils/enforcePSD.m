function matrixPSD = enforcePSD(matrix)
    if size(matrix,1) ~= size(matrix,2)
        error('Input matrix is not symmetric.');
    else
        matrixPSD = matrix;
        for r = 1:size(matrixPSD,1)
            for c = 1:size(matrixPSD,2)
                if r == c
                    matrixPSD(r,c) = abs(matrixPSD(r,c));
                else
                     offDiagElement = mean([matrixPSD(r,c),matrixPSD(c,r)]);
%                      offDiagElement = matrixPSD(r,c);
                     matrixPSD(c,r) = offDiagElement;
                     matrixPSD(r,c) = offDiagElement;
                end
            end
        end
    end
end