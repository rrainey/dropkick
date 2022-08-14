classdef GravityModel
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
  properties(Constant)
    re = 6378137.0;                      %% earth radius at equator, meters 
    GM = 0.3986004418e15;                %% m^3 s^-2 */
    J2 = 1.08263e-03;                    %% gravitational harmonic constant 
  endproperties
  methods(Static)
    function g_vec = ComputeGravityVectorEx(p_meters, dSinGeocentricLatitude)
        dSqr = dSinGeocentricLatitude * dSinGeocentricLatitude;
        dpMag_meters = norm(p_meters);
        dREPSqr = (GravityModel.re / dpMag_meters) * (GravityModel.re / dpMag_meters);
        p_bar = [ ...
            p_bar(1) = p_meters(1) * (1.0 + 1.5 * GravityModel.J2 * dREPSqr * (1.0 - 5.0 * dSqr));
            p_bar(2) = p_meters(2) * (1.0 + 1.5 * GravityModel.J2 * dREPSqr * (1.0 - 5.0 * dSqr));
            p_bar(3) = p_meters(3) * (1.0 + 1.5 * GravityModel.J2 * dREPSqr * (3.0 - 5.0 * dSqr));
        ];

        g_vec = p_bar * (-GravityModel.GM / (dpMag_meters * dpMag_meters * dpMag_meters));
    endfunction

    function g_vec = ComputeGravityVector(p_meters)
        g_vec = GravityModel.ComputeGravityVectorEx ( p_meters, p_meters(3) / norm(p_meters));
    endfunction
  endmethods
endclassdef