classdef Modeler
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

  methods(Static)

    function M = QToMatrix(Q)
      M = [ ...
        (Q(1)^2 + Q(2)^2 - Q(3)^2 - Q(4)^2), ...
        2 * ((Q(2)*Q(3)) + Q(1) * Q(4)), ...
        2 * ((Q(2)*Q(4)) - Q(1) * Q(3)) ; ...
        2 * ((Q(2)*Q(3)) - Q(1) * Q(4)), ...
        (Q(1)^2 + Q(3)^2 - Q(2)^2 - Q(4)^2), ...
        2 * ((Q(3)*Q(4)) + Q(1) * Q(2)); ...
        2 * ((Q(2)*Q(4)) + Q(1) * Q(3)), ...
        2 * ((Q(3)*Q(4)) - Q(1) * Q(2)), ...
        (Q(1)^2 + Q(4)^2 - Q(2)^2 - Q(3)^2) ...
      ];
    end

    function [phi, theta, psi] = QToEuler(Q)
      phi = atan(2 * (Q(3) * Q(4) + Q(1) * Q(2)) / (Q(1)^2 +Q(4)^2 -Q(2)^2-Q(3)^2));
      theta = asin( -2 * (Q(2) * Q(4) - Q(1) * Q(3)));
      psi = atan(2 * (Q(2) * Q(3) + Q(1) * Q(4)) / (Q(1)^2 +Q(2)^2 - Q(3)^2 - Q(4)^2));
    end

    function Q = EulerToQ(phi, theta, psi)
      Q = [ 
          cos(psi/2) * cos(theta/2) * cos(phi/2) + sin(psi/2) * sin(theta/2) * sin(phi/2); ...
          cos(psi/2) * cos(theta/2) * sin(phi/2) - sin(psi/2) * sin(theta/2) * cos(phi/2); ...
          cos(psi/2) * sin(theta/2) * cos(phi/2) + sin(psi/2) * cos(theta/2) * sin(phi/2); ...
        - cos(psi/2) * sin(theta/2) * sin(phi/2) + sin(psi/2) * cos(theta/2) * cos(phi/2); ...
        ];
    end

    function GeodeticToNED(lat_rad, lon_rad)
      north = [1; 0; 0];
    end

    function runit(path)

      IMUtoBody = [ 0 0 1 ; 1 0 0 ; 0 1 0 ];
      ts_ms = 0;
      ts_last_ms = -1;
      %% this could be done better: use initial gravity reading to set initial roll and pitch
      q = EulerToQ(0.0, 0.0, 0.0);

      %% 1) position and velocity expressed in meters & m/s in NED frame
      %% 2) position and velocity are continuously updated from the current GPS fix until
      %%    the jump starts.
      %% 3) A sudden "drop" signals the start of a jump.
      %% NED frame origin is the center of the landing area (790 ft MSL)
      %% SSD A/B target: 33.451283, -96.376357
      %% SSD C target: 33.451188, -96.378220
      %% SSD D target: 33.450053, -96.378935
      p_NED = [0; 0; 0];
      v_NED = [0; 0; 0];

      fid = fopen(path);
      tline = fgetl(fid);
      while ischar(tline)
          [data ier] = dropkick_nmealineread(tline);

            if (exist('data.x_acc','var') == 1)

              %% if first reading (== -1), we don't have enough info to
              %% integrate an update; just record t as t_last
              if (t_last_ms != -1)

                delta_t_ms = ts_ms - ts_last_ms;

                %% apply corrections

                %% value obtained from running IMU_zero on the device collecting
                %% the data
                %% setGyroRange(MPU6050_RANGE_250_DEG);
                %% setAccelerometerRange(MPU6050_RANGE_4_G);

                acc_corr_raw = [ -1012  -2581    1308 ];
                gyro_corr_raw = [ -37      28      -1 ];
                accel_scale = 8192;   %% sensor units to m/sec^2 (based on 4G scale)
                gyro_scale = 131;     %% sensor units to rad/sec (based on 250 deg/sec scale)


                %% apply corrections to linear acceleration and body rates

                acc = [ ( data.x_acc * accel_scale - acc_corr_raw(1) ) / accel_scale ...
                        ( data.y_acc * accel_scale - acc_corr_raw(2) ) / accel_scale ...
                        ( data.z_acc * accel_scale - acc_corr_raw(3) ) / accel_scale ...
                      ];

                [P, Q, R] = [ ...
                        ( data.x_rot * gyro_scale - gyro_corr_raw(1) ) / gyro_scale ...
                        ( data.y_rot * gyro_scale - gyro_corr_raw(2) ) / gyro_scale ...
                        ( data.z_rot * gyro_scale - gyro_corr_raw(3) ) / gyro_scale ...
                      ]; 

                %% Update orientation and position
                
                Omega = [0 P Q R ; ...
                      -P 0 -R Q; ...
                      -Q R 0 -P; ...
                      -R -Q P 0];
                
                %% update body orientation quaternion
                q_dot = - 0.5 * Omega * q;
                q = q + q_dot * delta_t_ms;

                g = [0; 0; 9.81]; %% m/sec^s

                B = QToMatrix(q);

                a_NED = B * (acc_sens + g);

                v_NED = v_NED + a_NED * delta_t_ms;

                p_NED = p_NED + v_NED * delta_t_ms;

              end

              ts_last_ms = ts_ms;
            end

          tline = fgetl(fid);
      end
      fclose(fid);
    endfunction

  endmethods
endclassdef