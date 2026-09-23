function cfg = loadConfig(file)
%LOADCONFIG Load a saved configuration, validate its version and fill missing fields.
%
% Inputs:
%   file - char, .mat file written by saveConfig                         [-]
% Outputs:
%   cfg  - struct, configuration; any field missing in the file is filled
%          recursively from initDefaults()                               [mixed]

%% ===== Load =====
S = load(file);
if ~isfield(S, 'cfg') || ~isstruct(S.cfg)
    error('loadConfig:format', 'File ''%s'' does not contain a cfg struct.', file);
end
cfg = S.cfg;

%% ===== Version check =====
supported = 1;   % [-] highest format_version understood by this build
if ~isfield(cfg, 'format_version')
    warning('loadConfig:noVersion', 'No format_version in ''%s''; assuming version 1.', file);
    cfg.format_version = 1;                              % [-] assume the first format version
elseif cfg.format_version > supported
    error('loadConfig:version', 'format_version %g is newer than supported (%d).', cfg.format_version, supported);
end

%% ===== Fill missing fields =====
cfg = localFill(cfg, initDefaults());
end

%% ===== Local functions =====
function a = localFill(a, d)
%LOCALFILL Recursively copy fields of d that are missing in a.
%
% Inputs:
%   a - struct, loaded struct                                             [mixed]
%   d - struct, default struct                                            [mixed]
% Outputs:
%   a - struct, a with missing fields taken from d                        [mixed]

fn = fieldnames(d);
for k = 1:numel(fn)
    f = fn{k};
    if ~isfield(a, f)
        a.(f) = d.(f);
    elseif isstruct(d.(f)) && isstruct(a.(f)) && isscalar(d.(f)) && isscalar(a.(f))
        a.(f) = localFill(a.(f), d.(f));
    end
end
end
