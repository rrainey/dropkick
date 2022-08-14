classdef GeodeticPosition
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
  properties
    Latitude_rad
    Longitude_rad
    Z_meters       %% meters above WGS-84 ellipsoid (no geoid correction)
  end
  properties(Constant)
    WGS84_MAJOR = 6378137.0;                  %% meters */
    WGS84_MINOR = 6356752.3142;               %% meters */
    WGS84_ECC =  0.081819190928906199466;     %% eccentricity */
    WGS84_ECC_SQR  = 0.006694380004260806515; %% eccentricity squared */
  end
  methods
    function obj = GeodeticPosition(lat, lon, z)
        if (nargin() > 0)
            obj.Latitude_rad = lat;
            obj.Longitude_rad = lon;
            obj.Z_meters = z;
        else
            obj.Latitude_rad = 0;
            obj.Longitude_rad = 0;
            obj.Z_meters = 0;
        end
    end

    %% Generate NED transform matrix centered on this position
    function out = NEDTransform(obj)

        out = eye(4);

        gc = GeocentricCoordinates([obj.Latitude_rad], [obj.Longitude_rad], [obj.Z_meters]);
        gcv = [ gc.X_m; gc.Y_m; gc.Z_m ];

        uz = - gcv / norm(gcv);
        unorth = [0 ; 0; 1.0];

        uy = cross(uz, unorth);
        ux = cross(uy, uz);

        out(1,1) = ux(1);
        out(1,2) = ux(2);
        out(1,3) = ux(3);

        out(2,1) = uy(1);
        out(2,2) = uy(2);
        out(2,3) = uy(3);

        out(3,1) = uz(1);
        out(3,2) = uz(2);
        out(3,3) = uz(3);

        out(1,4) = - gcv(1);
        out(2,4) = - gcv(2);
        out(3,4) = - gcv(3);
    end

    function out = GeocentricCoordinates(obj)

        out = [ 0; 0; 0 ];

        sin_latitude = sin(obj.Latitude_rad);
        cos_latitude = cos(obj.Latitude_rad);

        %%
        %%  N is the length of the normal line segment from the surface to the
        %%  spin axis.
        %%
        N = GeodeticPosition.WGS84_MAJOR / sqrt (1.0 - ...
            (GeodeticPosition.WGS84_ECC_SQR * sin_latitude * sin_latitude));

        %%
        %%  N1 lengthens the normal line to account for height above the surface
        %%
        N1 = N + obj.Z_meters;

        out(1) = N1 * cos_latitude * cos (obj.Longitude_rad);
        out(2) = N1 * cos_latitude * sin (obj.Longitude_rad);
        out(3) = (((GeodeticPosition.WGS84_MINOR * GeodeticPosition.WGS84_MINOR) ...
          / (GeodeticPosition.WGS84_MAJOR * GeodeticPosition.WGS84_MAJOR)) * N + obj.Z_meters) ...
          * sin_latitude;
    end

  end
end
