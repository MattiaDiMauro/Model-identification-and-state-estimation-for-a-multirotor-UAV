clc
clear
close all

%% Load data 

load('log_2_2024-5-22-17-28-50_2dd (1).mat')

% plot input
figure
u_time=x0.timestamp;
u_value=x0.value;
u_value=u_value(9797:16084);
u_time=u_time(9797:16084);
plot(u_time, u_value)
grid
title('input signal')
ylabel('u')
xlabel('time [s]')

%% Same rate

acc_body= acc.value;
acc_body_under=interp1(acc.timestamp,acc_body,u_time);
theta=interp1(theta.timestamp,theta.value,u_time);
pos=interp1(x.timestamp,x.value,u_time);

%% Plot output 1

figure
y_time_pos=u_time;
y_value_pos=pos;
plot(y_time_pos, y_value_pos)
grid
title('output signal')
ylabel('Position [m]')
xlabel('time [s]')

%% Plot output 2

acc_inertial=zeros(length(acc_body_under),3);

for i=1:length(acc_body_under)
    
    ROT=eul2rotm([0,theta(i),0]);
    acc_inertial(i,:)=ROT*acc_body_under(i,:)';
    
end

figure
y_time_acc=u_time;
y_value_acc=acc_inertial;

plot(y_time_acc, y_value_acc)
grid
legend('lin x acc','lin y acc','lin z acc')
title('output signal')
ylabel('Acc [m/s^2]')
xlabel('time [s]')

x_acc=y_value_acc(:,1);
y_value_pos=y_value_pos-y_value_pos(1);
x_acc=x_acc-x_acc(1);

%% Dataset 

Ts=median(diff(acc.timestamp));
data = iddata([y_value_pos,x_acc], u_value, Ts, 'Name', 'Closed-Loop');

data.InputName = 'SetPoint_Position';
data.Tstart = 0;
data.TimeUnit = 's';

%% Model definition

odefun = @SecondOrderModel;

omega_n = 1;
psi = 0.1;
G = 1;

parameters = {'omega_n',omega_n;'Psi',psi;'Gain',G};

fcn_type = 'c';

sys = idgrey(odefun,parameters,fcn_type);
sys.Structure.Parameters(3).Free = false;

%% Grey-box estimation

sys = greyest(data,sys)

%% Validation (PRBS)
load('log_3_2024-5-22-17-30-40_2dd (1).mat')

% Input PRBS
u_time_val = x0.timestamp;
u_value_val = x0.value;
u_value_val = u_value_val(7923:14424);
u_time_val  = u_time_val(7923:14424);

figure
plot(u_time_val, u_value_val)
grid
title('Validation input signal (PRBS)')
ylabel('u')
xlabel('time [s]')

% Bring data to same rate as input
acc_body_val = acc.value;
acc_body_val_under = interp1(acc.timestamp, acc_body_val, u_time_val);
theta_val = interp1(theta.timestamp, theta.value, u_time_val);
pos_val   = interp1(x.timestamp, x.value, u_time_val);

% Compute inertial accelerations for validation dataset

acc_inertial_val = zeros(length(acc_body_val_under),3);

for i = 1:length(acc_body_val_under)
    ROT = eul2rotm([0, theta_val(i), 0]);
    acc_inertial_val(i,:) = ROT * acc_body_val_under(i,:)';
end

x_acc_val = acc_inertial_val(:,1);

% Normalize signals
pos_val = pos_val - pos_val(1);
x_acc_val = x_acc_val - x_acc_val(1);

% Prepare validation time vector
u_time_val = (u_time_val - u_time_val(1));
u_time_val = linspace(u_time_val(1), u_time_val(end), length(u_time_val));

% Simulate grey-box model
[y_sim, t_sim] = lsim(sys, u_value_val, u_time_val);

% Plot comparison
figure

subplot(211)
plot(u_time_val, pos_val, 'r')
hold on
plot(t_sim, y_sim(:,1), 'b')
grid on
xlabel('time [s]')
ylabel('Position [m]')
legend('Measured','Simulated')
title('Validation - Position')

subplot(212)
plot(u_time_val, x_acc_val, 'r')
hold on
plot(t_sim, y_sim(:,2), 'b')
grid on
xlabel('time [s]')
ylabel('Acc [m/s^2]')
legend('Measured','Simulated')
title('Validation - Acceleration')
