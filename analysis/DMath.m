classdef DMath
%% Dropkick Log Analysis Project
%% Copyright (C) 2022 Riley Rainey

%% This program is free software: you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation, either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%%GNU General Public License for more details.

%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

  properties (Constant)
    Pi  = 3.14159265358979323846;
  endproperties
  methods (Static)
    function rad = DEGtoRAD(deg)
        rad = deg * DMath.Pi / 180.0;
    endfunction

    function deg = RADtoDEG(rad)
        deg = rad * 180.0 / DMath.Pi;
    endfunction
  endmethods
endclassdef