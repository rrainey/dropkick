%% Dropkick Logfile Analysis Toolkit
%% Riley Rainey riley@websimulations.com
%% Copyright (c) 2022, Riley Rainey
classdef DMath
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