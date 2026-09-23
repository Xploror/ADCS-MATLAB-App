function saveConfig(file, cfg)
%SAVECONFIG Save a user-level configuration to a MAT file (v7, MATLAB/Octave portable).
%
% Inputs:
%   file - char, target .mat file path                                   [-]
%   cfg  - struct, user-level configuration (initDefaults layout)        [mixed]
% Outputs:
%   (none; writes the file)

if ~isstruct(cfg)
    error('saveConfig:type', 'cfg must be a struct.');
end
if ~isfield(cfg, 'format_version')
    cfg.format_version = 1;                              % [-] config file format version written by this build
end
save(file, 'cfg', '-v7');
end
