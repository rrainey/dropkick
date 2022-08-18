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
      M = transpose(M);
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

    function [t_plot, a_norm_plot, a_filt_plot, sim] = analyze(path)

      %% 3x3 convert sensor frame to body frame
      %% TODO: compute dynamically from data while still in aircraft
      SensorToBody = [ 0 0 1 ; ...
                       1 0 0 ; ...
                       0 1 0 ];

      ts_ms = 0;
      ts_last_ms = -1;

      %% 1) position and velocity expressed in meters & m/s in NED frame
      %% 2) position and velocity are continuously updated from the current GPS fix until
      %%    the jump starts.
      %% 3) A sudden "drop" signals the start of a jump.

      %% this could be done better: use initial gravity reading to set initial roll and pitch
      q = Modeler.EulerToQ(0.0, 0.0, 0.0);

      p_NED = [0; 0; 0];
      v_NED = [0; 0; 0];
      a_filt = 9.81;
      a_filt_dot = [ 0 ; 0 ; 0 ];

      t_plot = [];
      a_norm_plot= [];
      a_filt_plot = [];

      t_jump = -1;
      t_deployment_start = -1;
      t_touchdown = -1;

      t_first_ms = -1;


      %% Skydive Spaceland Dallas - Student/A/B landing target (8/2022)
      %% Source: Google Maps
      %% 33.451283, -96.376360, 240.79
      %% NED frame origin is the center of the landing area (790 ft MSL)
      %% SSD A/B target: 33.451283, -96.376357
      %% SSD C target: 33.451188, -96.378220
      %% SSD D target: 33.450053, -96.378935

      ned_origin = GeodeticPosition( DMath.DEGtoRAD(33.451283), DMath.DEGtoRAD(-96.376360), 240.79 );
      ECFtoNED = ned_origin.NEDTransform();

      %% n_mode == 1 -- aircraft climbout
      %% n_mode == 2 -- freefall
      %% n_mode == 3 -- deployment started
      %% n_mode == 4 -- under canopy
      %% n_mode == 5 -- landed
      n_mode = 1;

      %% value obtained from running IMU_zero on the device collecting
      %% the data
      %% setGyroRange(MPU6050_RANGE_250_DEG);
      %% setAccelerometerRange(MPU6050_RANGE_4_G); (requires factoring calibration values by 0.5)

      acc_corr_raw = [ -1012/2  -2581/2    1308/2 ];
      gyro_corr_raw = [ -37      28      -1 ];
      accel_scale = 8192;   %% sensor units to m/sec^2 (based on 4G scale)
      gyro_scale = 131;     %% sensor units to rad/sec (based on 250 deg/sec scale)

      % Adafruit sensor package constants ...
      SENSORS_GRAVITY_EARTH = 9.80665;
      SENSORS_DPS_TO_RADS = 0.017453293;

      AFILT_TIME_CONSTANT = 2.0;

      fid = fopen(path);
      tline = fgetl(fid);
      while ischar(tline)

        [data ier] = dropkick_nmealineread(tline);

        if (ier == 0)

          %% aircraft climbout
          if (n_mode == 1)

            if (strcmp(data.type,'$GNVTG') == 1)
            %% VTG record
            %% update NED velocity
              groundspeed_mps = data.groundspeed.kph * 1000.0 / 3600.0;
              v_NED(1) = sin(DMath.DEGtoRAD(data.truecourse)) * groundspeed_mps;
              v_NED(2) = cos(DMath.DEGtoRAD(data.truecourse)) * groundspeed_mps;
              v_NED(3) = 0.0; % TODO - incorporate rate of climb (derived from filtered altitude reading from PENV record)

            elseif (strcmp(data.type,'$GNGGA') == 1)
            %% GGA record
            %% update NED location

              if (data.fix_available == 1)

                ecf_pos = GeocentricCoordinates(DMath.DEGtoRAD(data.latitude), DMath.DEGtoRAD(data.longitude), data.altitude);
                p_NED = [ ...
                  ECFtoNED(1,1) * ecf_pos.X_m + ECFtoNED(1,2) * ecf_pos.Y_m + ECFtoNED(1,3) * ecf_pos.Z_m + ECFtoNED(1,4) ; ...
                  ECFtoNED(2,1) * ecf_pos.X_m + ECFtoNED(2,2) * ecf_pos.Y_m + ECFtoNED(2,3) * ecf_pos.Z_m + ECFtoNED(2,4) ; ...
                  ECFtoNED(3,1) * ecf_pos.X_m + ECFtoNED(3,2) * ecf_pos.Y_m + ECFtoNED(3,3) * ecf_pos.Z_m + ECFtoNED(3,4) ; ...
                ];

              end

            elseif (strcmp(data.type,'$PIMU') == 1)
              %% PIMU record
              %% just look for freefall while we build a filtered acceleration normal

              delta_t_ms = data.ts_ms - ts_last_ms;
              delta_t_sec = delta_t_ms / 1000.0;

              %% apply corrections to reported MPU6050 sensor values

              acc_sensor = [ ...
                      ( data.x_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(1) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                      ( data.y_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(2) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                      ( data.z_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(3) ) * SENSORS_GRAVITY_EARTH / accel_scale ...
                    ];

              acc_body = SensorToBody * acc_sensor;

              a_norm = norm(acc_sensor);

              a_filt_dot(3) = AFILT_TIME_CONSTANT * (a_norm - a_filt);

              %% ABM Integration of acceleration filter
              %a_filt = a_filt + delta_t_sec / 12.0 * ( 5.0 * a_filt_dot(3) + 8.0 * a_filt_dot(2) - a_filt_dot(1) );

              a_filt = a_filt + delta_t_sec * a_filt_dot(3);

              a_filt_dot(1) = a_filt_dot(2);
              a_filt_dot(2) = a_filt_dot(3);

              t_plot(end+1) = data.ts_ms / 1000.0;
              a_filt_plot(end+1) = a_filt;
              a_norm_plot(end+1) = a_norm;

              %%% mark start of jump as 15 seconds before approximate start of freefall
              if (a_filt < 6.00 && t_jump < 0.0)
                  t_jump = data.ts_ms - 15000;
                  n_mode = 2;

                  fprintf(1, 'IMU switch to mode 2 at %d (seconds) \n', (data.ts_ms - 15000) / 1000);
              end

              ts_last_ms = data.ts_ms;

            end

          else
            %% All other modes ...

            if (strcmp(data.type,'$PIMU') == 1)

              %% if first reading (== -1), we don't have enough info to
              %% integrate an update; just record t as t_last
              if (ts_last_ms != -1)

                delta_t_ms = data.ts_ms - ts_last_ms;
                delta_t_sec = delta_t_ms / 1000.0;

                %% apply corrections to reported MPU6050 sensor values

                 acc_sensor = [ ...
                      ( data.x_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(1) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                      ( data.y_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(2) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                      ( data.z_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(3) ) * SENSORS_GRAVITY_EARTH / accel_scale ...
                    ];


                acc_body = SensorToBody * acc_sensor;

                a_norm = norm(acc_sensor);

                a_filt_dot(3) = AFILT_TIME_CONSTANT * (a_norm - a_filt);

                %% ABM Integration of acceleration filter
                %a_filt = a_filt + delta_t_sec / 12.0 * ( 5.0 * a_filt_dot(3) + 8.0 * a_filt_dot(2) - a_filt_dot(1) );

                a_filt = a_filt + delta_t_sec * a_filt_dot(3);

                a_filt_dot(1) = a_filt_dot(2);
                a_filt_dot(2) = a_filt_dot(3);

                t_plot(end+1) = data.ts_ms / 1000.0;
                a_filt_plot(end+1) = a_filt;
                a_norm_plot(end+1) = a_norm;

                if (a_filt > 18.0 && t_deployment_start < 0.0)
                  t_deployment_start = data.ts_ms;
                  n_mode = 3;

                  fprintf(1, 'IMU switch to mode 3 at %d (seconds) \n', (data.ts_ms - 15000) / 1000);
                end

                %% axis rotation rates in sensor frame
                rot_sensor = [( data.x_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(1) ) / gyro_scale * SENSORS_DPS_TO_RADS ;...
                              ( data.y_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(2) ) / gyro_scale * SENSORS_DPS_TO_RADS ;...
                              ( data.z_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(3) ) / gyro_scale * SENSORS_DPS_TO_RADS ]; 

                %% axis rotation rates in body frame
                a = SensorToBody * rot_sensor;

                P = a(1);
                Q = a(2);
                R = a(3);

                %% Update orientation 
                Omega_b = [0.0   P    Q    R ; ...
                          -P   0.0   -R    Q; ...
                          -Q     R  0.0   -P; ...
                          -R    -Q    P  0.0];
                
                %% update body orientation quaternion
                q_dot = - 0.5 * Omega_b * q;
                q = q + q_dot * delta_t_sec;

                g_NED = [0; 0; 9.814]; %% m/sec^s

                B = Modeler.QToMatrix(q);

                a_NED = B * acc_body + g_NED;

                %% Euler integration (TODO: switch to ABM)

                v_NED = v_NED + a_NED * delta_t_sec;

                p_NED = p_NED + v_NED * delta_t_sec;

              end

              ts_last_ms = data.ts_ms;

              if (t_first_ms < 0.0)
                t_first_ms = data.ts_ms;
              end
            end
          end

        end

        tline = fgetl(fid);
      end
      %% 
      %% End of pass 1
      %%
      fprintf(1,...
            '\n\tEnd of Pass 1; t_jump: %f seconds\nt_deplotment: %f seconds\nt_touchdown: %f seconds\n',...
            t_jump/1000, t_deployment_start/1000, t_touchdown/1000);

      fseek (fid,0);
      pass2_complete = 0;

      tline = fgetl(fid);
      while (ischar(tline) && pass2_complete == 0)

        %% Seek to just prior to the jump

        [data ier] = dropkick_nmealineread(tline);

        if (ier == 0)

          if (strcmp(data.type,'$GNVTG') == 1)
            %% VTG record
            %% update NED velocity
            groundspeed_mps = data.groundspeed.kph * 1000.0 / 3600.0;
            v_NED(1) = sin(DMath.DEGtoRAD(data.truecourse)) * groundspeed_mps;
            v_NED(2) = cos(DMath.DEGtoRAD(data.truecourse)) * groundspeed_mps;
            v_NED(3) = 0.0; % TODO - incorporate rate of climb (derived from filtered altitude reading from PENV record)

          elseif (strcmp(data.type,'$GNGGA') == 1)
            %% GGA record
            %% update NED location

            if (data.fix_available == 1)

              ecf_pos = GeocentricCoordinates(DMath.DEGtoRAD(data.latitude), DMath.DEGtoRAD(data.longitude), data.altitude);
              p_NED = [ ...
                ECFtoNED(1,1) * ecf_pos.X_m + ECFtoNED(1,2) * ecf_pos.Y_m + ECFtoNED(1,3) * ecf_pos.Z_m + ECFtoNED(1,4) ; ...
                ECFtoNED(2,1) * ecf_pos.X_m + ECFtoNED(2,2) * ecf_pos.Y_m + ECFtoNED(2,3) * ecf_pos.Z_m + ECFtoNED(2,4) ; ...
                ECFtoNED(3,1) * ecf_pos.X_m + ECFtoNED(3,2) * ecf_pos.Y_m + ECFtoNED(3,3) * ecf_pos.Z_m + ECFtoNED(3,4) ; ...
              ];

            end

          elseif (strcmp(data.type,'$PIMU') == 1)
            %% PIMU record
            %% just look for freefall while we build a filtered acceleration normal

            delta_t_ms = data.ts_ms - ts_last_ms;
            delta_t_sec = delta_t_ms / 1000.0;

            %% apply corrections to reported MPU6050 sensor values

            acc_sensor = [ ...
                    ( data.x_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(1) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                    ( data.y_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(2) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                    ( data.z_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(3) ) * SENSORS_GRAVITY_EARTH / accel_scale ...
                  ];

            acc_body = SensorToBody * acc_sensor;

            a_norm = norm(acc_sensor);

            a_filt_dot(3) = AFILT_TIME_CONSTANT * (a_norm - a_filt);

            %% ABM Integration of acceleration filter
            %a_filt = a_filt + delta_t_sec / 12.0 * ( 5.0 * a_filt_dot(3) + 8.0 * a_filt_dot(2) - a_filt_dot(1) );

            a_filt = a_filt + delta_t_sec * a_filt_dot(3);

            a_filt_dot(1) = a_filt_dot(2);
            a_filt_dot(2) = a_filt_dot(3);

            ts_last_ms = data.ts_ms;

            if (data.ts_ms > t_jump)
              pass2_complete = 1;
            end
          end
        end

        tline = fgetl(fid);
      end

      %%
      %% Pass 3 integrate IMU sensor information
      %%
      sim = struct([]);
      tline = fgetl(fid);
      while ischar(tline)

        [data ier] = dropkick_nmealineread(tline);

        if (ier == 0)

          if (strcmp(data.type,'$PIMU') == 1)
            %% PIMU record
            %% just look for freefall while we build a filtered acceleration normal

            delta_t_ms = data.ts_ms - ts_last_ms;
            delta_t_sec = delta_t_ms / 1000.0;

            %% apply corrections to reported MPU6050 sensor values

            acc_sensor = [ ...
                    ( data.x_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(1) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                    ( data.y_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(2) ) * SENSORS_GRAVITY_EARTH / accel_scale ; ...
                    ( data.z_acc / SENSORS_GRAVITY_EARTH * accel_scale - acc_corr_raw(3) ) * SENSORS_GRAVITY_EARTH / accel_scale ...
                  ];

            acc_body = SensorToBody * acc_sensor;

            a_norm = norm(acc_sensor);

            a_filt_dot(3) = AFILT_TIME_CONSTANT * (a_norm - a_filt);

            %% ABM Integration of acceleration filter
            %a_filt = a_filt + delta_t_sec / 12.0 * ( 5.0 * a_filt_dot(3) + 8.0 * a_filt_dot(2) - a_filt_dot(1) );

            a_filt = a_filt + delta_t_sec * a_filt_dot(3);

            a_filt_dot(1) = a_filt_dot(2);
            a_filt_dot(2) = a_filt_dot(3);

            % axis rotation rates in sensor frame
            rot_sensor = [( data.x_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(1) ) / gyro_scale * SENSORS_DPS_TO_RADS ;...
                          ( data.y_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(2) ) / gyro_scale * SENSORS_DPS_TO_RADS ;...
                          ( data.z_rot / SENSORS_DPS_TO_RADS * gyro_scale  - gyro_corr_raw(3) ) / gyro_scale * SENSORS_DPS_TO_RADS ]; 

            %% axis rotation rates in body frame
            a = SensorToBody * rot_sensor;

            P = a(1);
            Q = a(2);
            R = a(3);

            %% Update orientation 

            %K_epsilion = 10.0 * (1.0 - (q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2));

            Omega_b = [0.0   P     Q    R ; ...
                        -P   0.0  -R    Q; ...
                        -Q   R   0.0   -P; ...
                        -R   -Q    P  0.0];

            % From ANSI/AIAA R-004-1992 E.2.2:
            
            %q_dot = [ ...
            %       - 0.5 * (q(2) * P + q(3) * Q + q(4) * R ) + K_epsilion * q(1); ...
            %         0.5 * (q(1) * P + q(3) * R - q(4) * Q ) + K_epsilion * q(2); ...
            %         0.5 * (q(1) * Q + q(4) * P - q(2) * R ) + K_epsilion * q(3); ...
            %         0.5 * (q(1) * R + q(2) * Q - q(3) * P ) + K_epsilion * q(4) ...
            %];

            %% update body orientation quaternion
            q_dot = - 0.5 * Omega_b * q;
            q = q + q_dot * delta_t_sec;

            g_NED = [0; 0; 9.814]; %% m/sec^s

            B = Modeler.QToMatrix(q);

            a_NED = B * acc_body + g_NED;

            %% Euler integration (TODO: switch to ABM)

            v_NED = v_NED + a_NED * delta_t_sec;

            p_NED = p_NED + v_NED * delta_t_sec;

            ts_last_ms = data.ts_ms;

            element.t = data.ts_ms / 1000.0;
            element.q = q;
            [phi, theta, psi] = Modeler.QToEuler(q);
            element.euler = [ DMath.RADtoDEG(phi); DMath.RADtoDEG(theta); DMath.RADtoDEG(psi) ];
            element.v = v_NED;
            element.p = p_NED;
            sim(end+1) = element;
            clear element;

          end

        end
        tline = fgetl(fid);
      end

      fclose(fid);

    endfunction

    function plot_util(t_plot, a_norm, a_filt)
      hf = figure ();
      plot(t_plot, a_norm);
      hold on;
      plot(t_plot, a_filt, "r");
      xlabel("time(s)");
      ylabel("m/sec^2");
      title ("acceleration forces");
      print (hf, "acc_plot.png", "-dpng");
    endfunction
  endmethods
endclassdef