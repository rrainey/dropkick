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

%% 3x3 convert sensor frame to body
%% TODO: compute dynamically from data while still in aircraft
SensorToBody = [ 0 0 1 ; 1 0 0 ; 0 1 0 ];

ts_ms = 0;
ts_last_ms = -1;

%% this could be done better: use initial gravity reading to set initial roll and pitch
q = EulerToQ(0.0, 0.0, 0.0);
p_NED = [0; 0; 0];
v_NED = [0; 0; 0];
a_filt_dot = [ 9.81 ; 9.81 ; 9.81 ];

x_plot = [];
y_plot = [];

t_jump = -1;
t_deployment_start = -1;
t_touchdown = -1;

%% Skydive Spaceland Dallas - Student/A/B landing target (8/2022)
%% Source: Google Maps
%% 33.451283, -96.376360, 240.79

ned_origin = GeodeticPosition( DEGtoRAD(33.451283), DEGtoRAD(-96.376360), 240.79 );
ECFtoNED = ned_origin.NEDTransform();

%% n_mode == 1 -- aircraft climbout
%% n_mode == 2 -- freefall
%% n_mode == 3 -- deployment started
%% n_mode == 4 -- under canopy
%% n_mode == 5 -- landed
n_mode = 1

fid = fopen('LOG00014.TXT');
tline = fgetl(fid);
while ischar(tline)
    [data ier] = dropkick_nmealineread(tline);

    %% aircraft climbout
    if (n_mode == 1)

      if (exist('data.truecourse','var') == 1 & n_mode == 1)
      %% VTG record
      %% update NED velocity
        groundspeed_mps = data.groundspeed.kph * 1000.0 / 3600.0;
        v_NED(0) = sin(DEGtoRAD(data.truecourse)) * groundspeed_mps;
        v_NED(1) = cos(DEGtoRAD(data.truecourse)) * groundspeed_mps;
        v_NED(3) = 0.0; // TODO - incorporate rate of climb (derived from filtered altitude reading from PENV record)

      elseif (exist('data.latitude','var') == 1 & exist('data.altitude','var') == 1 & n_mode == 1)
      %% GGA record
      %% update NED location

        ecf_pos = GeocentricCoordinates(DEGtoRAD(data.latitude), DEGtoRAD(data.longitude), data.altitude);
        p_NED = [ ...
          ECFtoNED(1,1) * ecf_pos(1) + ECFtoNED(1,2) * ecf_pos(2) + ECFtoNED(1,3) * ecf_pos(3) + ECFtoNED(1,4) ; ...
          ECFtoNED(2,1) * ecf_pos(1) + ECFtoNED(2,2) * ecf_pos(2) + ECFtoNED(2,3) * ecf_pos(3) + ECFtoNED(2,4) ; ...
          ECFtoNED(3,1) * ecf_pos(1) + ECFtoNED(3,2) * ecf_pos(2) + ECFtoNED(3,3) * ecf_pos(3) + ECFtoNED(3,4) ; ...
        ];
     

      elseif (exist('data.x_acc','var') == 1)
      %% PIMU record
      %% just look for freefall while we build a filtered acceleration normal

        a_sense = [ data.x_acc ; data.y_acc ; data.z_acc ];
        a_norm = norm(a_sense);

        a_filt_dot(3) = 1.0 * (a_norm - a_filt);

        delta_t_sec = (data.ts_ms - ts_last_ms) / 1000.0;

        %% ABM Integration of acceleration filter
        a_filt = a_filt + delta_t_sec / 12.0 * ( 5.0 * a_filt_dot(3) + 8.0 * a_filt_dot(2) - a_filt_dot(1) );

        a_filt_dot(1) = a_filt_dot(2);
        a_filt_dot(2) = a_filt_dot(3);

        x_plot(end) = data.t_ms / 1000.0;
        y_plot(end) = a_filt;

        %%% mark start of jump as 15 seconds before approximate start of freefall
        if (a_filt < 0.5 & t_jump > 0) {
            t_jump = data.t_ms - 15000;
        }

      end

    else
      %% All other modes ...

      if (exist('data.x_acc','var') == 1)
      %% PIMU record
      %% execute pose/location integration step or just look for freefall

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

          %% apply corrections to reported MPU6050 sensor values
          %% todo: transform from sensor to "body" frame

          acc_sensor = [ ( data.x_acc * accel_scale - acc_corr_raw(1) ) / accel_scale ...
                  ( data.y_acc * accel_scale - acc_corr_raw(2)) ) / accel_scale ...
                  ( data.z_acc * accel_scale - acc_corr_raw(3)) ) / accel_scale ...
                ];

          acc_body = SensorToBody * acc_sensor;

          %% axis rotation rates in sensor frame
          rot_sensor = [( data.x_rot * gyro_scale - gyro_corr_raw(1) ) / gyro_scale ;...
                        ( data.y_rot * gyro_scale - gyro_corr_raw(2) ) / gyro_scale ;...
                        ( data.z_rot * gyro_scale - gyro_corr_raw(3) ) / gyro_scale]; 

          %% axis rotation rates in body frame
          [P, Q, R] = SensorToBody * rot_sensor;

          %% Update orientation 
          
          Omega_b = [0 P Q R ; ...
                  -P 0 -R Q; ...
                  -Q R 0 -P; ...
                  -R -Q P 0];
          
          %% update body orientation quaternion
          q_dot = - 0.5 * Omega_b * q;
          q = q + q_dot * delta_t_ms / 1000.0;

          g = [0; 0; 9.81]; %% m/sec^s

          B = QToMatrix(q);

          a_NED = B * (acc_sens) + g;

          %% Euler integration (TODO: switch to ABM)

          v_NED = v_NED + a_NED * delta_t_ms / 1000.0;

          p_NED = p_NED + v_NED * delta_t_ms / 1000.0;

        end

        ts_last_ms = ts_ms;
      end
    end

    tline = fgetl(fid);
end
fclose(fid);