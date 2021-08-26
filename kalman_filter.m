% Kalman Filter (Euler Angles)

%% SETUP
clear;
clc;

movement_file = 'pitch_imu.csv';
stationary_file = 'parado_imu.csv';
%ground_truth_file = '';

data = csvread(movement_file);
calib = csvread(stationary_file);
%ground_truth = csvread(ground_truth_file);

accx = data(:,4);
accy = data(:,5);
accz = data(:,6);

gyro_x = data(:,1);
gyro_y = data(:,2);

len = length(accx);
f = 100;
dt = 1/f;

%% CALIBRATION
calib_acc = [calib(:,4) calib(:,5) calib(:,6)];
calib_gyro = [calib(:,1) calib(:,2)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

accx = accx - mean_calib_acc(1);
accy = accy - mean_calib_acc(2);
accz = accz + (9.8 - mean_calib_acc(3));

%% ACCEL ANGLES
acc_pitch = zeros(len,1);
acc_roll = zeros(len,1);

for i=1:len
   acc_pitch(i) = -atan2(accx(i),sqrt(accz(i)*accz(i)+accy(i)*accy(i)));
   acc_roll(i) = atan2(accy(i),sqrt(accz(i)*accz(i)+accx(i)*accx(i)));
end

%% KALMAN FILTER SETUP
X = zeros(len,2);
x = [0 0]';
A = [1 0
     0 1];
B = [dt 0
     0  dt];

var_q = [var(calib_gyro(:,1)) 0
                0          var(calib_gyro(:,2))];
Q = B*var_q*B';
P = eye(2);

R = [var(calib_acc(:,1)) 0
            0         var(calib_acc(:,2))];
        
%% KALMAN

for i=1:len
   w = [gyro_x gyro_y]';
   
   x_ = A*x + B*w;
   P_ = A*P*A' + Q;
   
   y_m = [acc_roll(i) acc_pitch(i)]';
   
   K = P*(P + R)^-1;
   
   x = x_ + K*(y_m - x_);
   
   P = (eye(2) - K)*P;
   
   X(i,1) = x(1);
   X(i,2) = x(2);
end

