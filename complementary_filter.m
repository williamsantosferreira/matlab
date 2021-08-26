% Complementary filter

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

%% GYRO ANGLES
gyro_pitch = zeros(len,1);
gyro_roll = zeros(len,1);

for i=2:len
   gyro_pitch(i) = gyro_pitch(i-1) + gyro_y(i)*dt;
   gyro_roll(i) = gyro_roll(i-1) + gyro_x(i)*dt;
end

%% COMPLEMENTARY FILTER
cf_pitch = zeros(len,1);
cf_roll = zeros(len,1);

a = 0.98;

for i=2:len
   cf_pitch(i) = a*(cf_pitch(i-1)+gyro_pitch(i)-gyro_pitch(i-1))+(1-a)*acc_pitch(i);  
   cf_roll(i) = a*(cf_roll(i-1)+gyro_roll(i)-gyro_roll(i-1))+(1-a)*acc_roll(i); 
end

%% PLOTING

subplot(1,2,1)
plot(cf_pitch);
title('PITCH');

subplot(1,2,2)
plot(cf_roll);
title('ROLL');
