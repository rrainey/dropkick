%% Earth Centered, Earth Fixed coordinates
classdef NEDCoordinates
  properties
    X_m %% North
    Y_m %% East
    Z_m %% Down
  end
  methods
    function obj = NEDCoordinates(x,y,z)
        if nargin > 0
            obj.X_m = x;
            obj.Y_m = y;
            obj.Z_m = z;
        else
            obj.X_m = 0.0;
            obj.Y_m = 0.0;
            obj.Z_m = 0.0;
        end
    end
  end
end