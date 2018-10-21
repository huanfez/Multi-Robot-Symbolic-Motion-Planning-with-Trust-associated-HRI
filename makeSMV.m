function[] = makeSMV(fileName, gridWidth, gridLength, start, obstacles, ltlspec)
%function[] = makeSMV(fileName, gridWidth, gridLength, start, obstacles)
%   fileName = string title of SMV, i.e. 'fileName.smv'
%   gridWidth = number of cells along x
%   gridLength = number of cells along y
%   start = initial cell
%   obstacles = vector of forbidden cells

fid = fopen(fileName,'w'); % open file

fprintf(fid, 'MODULE main\n');
fprintf(fid, 'VAR\n');
fprintf(fid, 'x : grid;\n');
fprintf(fid, '%s\n',ltlspec); % Write specification from input
fprintf(fid, 'MODULE grid\n');
fprintf(fid, 'VAR\n');
fprintf(fid, 'ASSIGN\n');
fprintf(fid, 'init(state) := %.0f;\n',start); % starting position
fprintf(fid, 'next(state) :=\n');
fprintf(fid, 'case\n');
printGridTransitions(fid,gridWidth, gridLength, obstacles);
% Run function to generate state transition rules
fprintf(fid, 'TRUE : state;\n');
fprintf(fid, 'esac;');

fclose(fid); % close file

















