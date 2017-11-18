function map = getMap(mapName)
% wrapper function to load a map structure from memory. This is a read for
% what the saved map structure should contain:
% Output:
%   map - structure containing all of the map information
%       .name - string with name of map
%       .dim - number of dimensions (this is in world coordinates, not
%              configuration space)
%       .min - vector of minimum values in each dimension
%       .max - vector of maximum values in each dimension
%       .N - vector of number of grid points for each dimension
%       .target - struct defining the target region
%           .radius - radius of target (for circular target, characteristic
%                     length for other shapes, ex: half of the width of a
%                     square)
%           .center - coordinates of center of target
%           .data - data matrix defining target for value function (ex:
%                   shapeSphere, shapeCylinder, etc.)   
%       .obstacles - static obstacle data for the map
%

if nargin >= 1
    map = load(['./DifferentialGames/Maps/',mapName]);
else
    error(['getMap must be called with a valid map name. ',... 
        'Valid map names are:\n\t%s'],ls('./DifferentialGames/Maps/*.mat'));
end