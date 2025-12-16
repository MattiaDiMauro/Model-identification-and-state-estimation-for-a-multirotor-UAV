%% Filter design for velocity estimate

% clc
% clear
% close all

%% Load data

%load('log_2_2024-5-22-17-28-50_2dd (1).mat')
load('log_3_2024-5-22-17-30-40_2dd (1).mat')

% Time
t = linspace(q.timestamp(1), q.timestamp(end), length(q.timestamp));
dt = 1/50;

% Constants
g = 9.81;

% Measurements
q_meas = interp1(q.timestamp, q.value, t);          % pitch rate [rad/s]
acc_x  = interp1(acc.timestamp, acc.value(:,1), t); % body x accel

% Filter initialization
theta0 = 0;
P0 = 1e-2;

theta_prev = theta0;
P_prev = P0;

% Noise
Q = (var(v.value(3370:5728)))^2;      % process noise (gyro integration)
R = (var(acc_x(7923:14424)))^2;     % accelerometer noise 


%% Loop
A_d = 1; 
B_d = dt;
C_d = g; 

acc_trans = [0 diff(v.value(:,1)')/dt];   % translational acceleration
acc_trans = interp1(v.timestamp, acc_trans, t);

theta_est = zeros(1, length(t));
acc_est   = zeros(1, length(t));


for k = 1:length(t)

    % PREDICTION 
    x_hat = A_d * theta_prev + B_d * q_meas(k);
    P_hat = A_d * P_prev * A_d' + Q; 

    % MEASUREMENT
    u_time_val = linspace(x0.timestamp(1), x0.timestamp(end), length(x0.timestamp));
    [y_sim, t_sim] = lsim(sys, x0.value, u_time_val);
    acc_est=y_sim(:,1);
    z = acc_x(k)-acc_est(k);   

    % CORRECTION 
    S = C_d * P_hat * C_d' + R;
    K = P_hat * C_d * inv(S);

    x = x_hat + K * (z - C_d * x_hat);
    P = (1 - K * C_d) * P_hat;

    theta_prev = x;
    P_prev = P;
    theta_est(k) = x;
end

%% Plot
figure
plot(t, theta_est, 'b')
hold on
plot(theta.timestamp, theta.value, 'r')
legend('filter', 'ground truth')
title('Pitch angle')
grid on


