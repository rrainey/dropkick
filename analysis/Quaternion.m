classdef Quaternion
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
    q;
  end
  methods
    %% Generate a matrix suitable to convert from world coordinates to
    %% body coordinates
    %% See E.2.4 in ANSI/AIAA R-004-1992
    function M = ToMatrix(obj)
      M = transpose( [ ...
        (obj.q(1)^2 + obj.q(2)^2 - obj.q(3)^2 - obj.q(4)^2), ...
        2 * ((obj.q(2)*obj.q(3)) + obj.q(1) * obj.q(4)), ...
        2 * ((obj.q(2)*obj.q(4)) - obj.q(1) * obj.q(3)) ; ...
        2 * ((obj.q(2)*obj.q(3)) - obj.q(1) * obj.q(4)), ...
        (obj.q(1)^2 + obj.q(3)^2 - obj.q(2)^2 - obj.q(4)^2), ...
        2 * ((obj.q(3)*obj.q(4)) + obj.q(1) * obj.q(2)); ...
        2 * ((obj.q(2)*obj.q(4)) + obj.q(1) * obj.q(3)), ...
        2 * ((obj.q(3)*obj.q(4)) - obj.q(1) * obj.q(2)), ...
        (obj.q(1)^2 + obj.q(4)^2 - obj.q(2)^2 - obj.q(3)^2) ...
      ]);
    end

    %% Extract Euler angles from Quaternion object
    %% See E.2.3 in ANSI/AIAA R-004-1992
    function [phi, theta, psi] = ToEuler(obj)
      phi = atan2(2 * (obj.q(3) * obj.q(4) + obj.q(1) * obj.q(2)), (obj.q(1)^2 +obj.q(4)^2 -obj.q(2)^2-obj.q(3)^2));
      theta = asin( -2 * (obj.q(2) * obj.q(4) - obj.q(1) * obj.q(3)));
      psi = atan2(2 * (obj.q(2) * obj.q(3) + obj.q(1) * obj.q(4)), (obj.q(1)^2 +obj.q(2)^2 - obj.q(3)^2 - obj.q(4)^2));
      if (psi < 0.0)
        psi += 2.0 * DMath.Pi;
      end
    end

    %% Initialize from AIAA Euler body angles
    %% See E.2.1 in ANSI/AIAA R-004-1992
    function obj = Quaternion(phi, theta, psi, extra)
      switch nargin
      case 4
        obj.q = [ phi; theta; psi; extra ];
      case 3
        %% assumes Euler angles: phi, theta, psi (radians)
        obj.q = [ 
            cos(psi/2) * cos(theta/2) * cos(phi/2) + sin(psi/2) * sin(theta/2) * sin(phi/2); ...
            cos(psi/2) * cos(theta/2) * sin(phi/2) - sin(psi/2) * sin(theta/2) * cos(phi/2); ...
            cos(psi/2) * sin(theta/2) * cos(phi/2) + sin(psi/2) * cos(theta/2) * sin(phi/2); ...
          - cos(psi/2) * sin(theta/2) * sin(phi/2) + sin(psi/2) * cos(theta/2) * cos(phi/2); ...
          ];
      case 1
        %% assume it's a 4-vector
        obj.q = [ phi(1); phi(2); phi(3); phi(4) ];
      otherwise
        obj.q = [ 1; 0; 0; 0 ];
      end
    end

    % Overload matrix multiplication
    function p = mtimes(obj1, obj2)
      p = Quaternion(obj1 * obj2.vec());
    end

    % Overload binary addition
    function p = plus(obj1, obj2)
      p = Quaternion(obj1.vec() + obj2.vec());
    end

    function vec = vec(obj)
      vec = obj.q;
    end

    % Overload display operator
    function disp(obj)
      printf(" [ ");
      for i = 1 : 4;
        printf("%.5g ", obj.q(i));
      end
      printf("] ");
      printf ("\n");
    end

  endmethods
endclassdef