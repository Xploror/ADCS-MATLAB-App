function exportMetricsCSV(file, header, rows)
% Write a metrics table (from metricsTable) to a CSV file.
%
% Inputs:
%   file   - char, output file path                                       
%   header - cell [1xC] of char, column titles                            
%   rows   - cell [KxC], char or double entries                           [mixed]
% Outputs:
%   (none; writes the file, errors if it cannot be opened)

%% ===== Open file =====
fid = fopen(file, 'w');
if fid < 0
    error('exportMetricsCSV:open', 'Cannot open ''%s'' for writing.', file);
end
cleaner = onCleanup(@() fclose(fid));

%% ===== Header and rows =====
fprintf(fid, '%s\n', localJoin(header));
for k = 1:size(rows, 1)
    fprintf(fid, '%s\n', localJoin(rows(k, :)));
end
end

%% ===== Local functions =====
function line = localJoin(cells)
% Join cell entries into one CSV line (chars quoted, numbers %.10g).
%
% Inputs:
%   cells - cell [1xC], char or numeric scalars                           [mixed]
% Outputs:
%   line  - char, comma-separated line                                    

%% ===== Format each cell =====
dq = char(34);                                          %  double-quote character (CSV text delimiter)
parts = cell(1, numel(cells));
for c = 1:numel(cells)
    v = cells{c};
    if ischar(v)
        parts{c} = [dq strrep(v, dq, [dq dq]) dq];       % quote text, escape embedded quotes
    elseif islogical(v)
        parts{c} = sprintf('%d', v);
    elseif isempty(v)
        parts{c} = '';
    else
        parts{c} = sprintf('%.10g', v);
    end
end
line = strjoin(parts, ',');
end
