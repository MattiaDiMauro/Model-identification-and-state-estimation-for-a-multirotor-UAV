clc
close all
clear

%% Load .mat

addpath('/home/ant-x/ANT-X_tools/DroneCmd/util/../log_mat')
load('2021-12-01-16-11-08');

%% Plot data 

figure
plot(acc.timestamp,acc.value)
grid on
xlabel('Time [s]')
ylabel('Acceleration in body frame [m/s^2]')
legend('a_x','a_y','a_z')

figure
plot(dist.timestamp,dist.value)
grid on
xlabel('Time [s]')
ylabel('Range sensor [m]')

figure
plot(theta.timestamp,theta.value)
grid on
xlabel('Time [s]')
ylabel('\theta [rad]')

figure
plot(x.timestamp,x.value)
grid on
xlabel('Time [s]')
ylabel('Position [m]')

figure
plot(x0.timestamp,x0.value)
grid on
xlabel('Time [s]')
ylabel('Setpoint position [m]')

figure
plot(x.timestamp,x.value)
hold on
plot(x0.timestamp,x0.value)
grid on
xlabel('Time [s]')
ylabel('Setpoint position vs Measured position [m]')
legend('Measurement','SetPoint')

figure
plot(v.timestamp,v.value)
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
