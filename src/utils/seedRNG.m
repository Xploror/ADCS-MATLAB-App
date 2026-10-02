function seedRNG(seed)
% Seed the global random generators (MATLAB rng / Octave rand,randn state).
%
% Inputs:
%   seed - double [1], non-negative integer seed
% Outputs:
%   (none)
%
% Not code-generation compatible (only used in MATLAB/Octave-level code).
% Note: MATLAB and Octave generate different sequences from the same seed.

if isOctave()
    rand('state', seed);   %#ok<RAND>
    randn('state', seed);  %#ok<RAND>
else
    rng(seed, 'twister');
end
end
