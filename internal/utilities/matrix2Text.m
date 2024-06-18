function matrix2Text(matrix, filename)
    %%
    % MATRIX2TEXT Writes the values of a matrix to a text file.
    % 
    %   matrixToText(matrix, filename) writes the values of 'matrix' to
    %   a text file specified by 'filename'.
    %
    %   Example:
    %       A = [1, 2, 3; 4, 5, 6; 7, 8, 9];
    %       matrixToText(A, 'matrix.txt');
    %%
    fileID = fopen(filename, 'w');
    if fileID == -1
        error('Cannot open file: %s', filename);
    end
    [rows, cols] = size(matrix);
    for i = 1:rows
        for j = 1:cols
            fprintf(fileID, '  %g  ', matrix(i, j));
        end
        fprintf(fileID, '\n');
    end
    fclose(fileID);
end