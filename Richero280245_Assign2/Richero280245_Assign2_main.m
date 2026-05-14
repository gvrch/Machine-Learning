%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% S3 – Assignment #2: From MIL to PIL
% Author: Giovanni Richero (ID: 280245)
% Course: Space Systems Engineering – Politecnico di Milano
%
% Simulink model: Richero280245_Assign2.slx
% Description:
%
% This script supports Assignment #2 of the Space Systems Simulation course.
% It performs the Model-In-the-Loop (MIL), and presents the plot of the results
% of Software-In-the-Loop (SIL), and Processor-In-the-Loop (PIL) analyses for 
% the nadir-pointing attitude control using the provided protected plant model.
%
% This script initializes  the spacecraft parameters, controller gains, and
% initial conditions, executes the required simulations, and generates the
% performance metrics requested in the assignment, including pointing error,
% quaternion error, angular velocity error, and reaction wheel control torque.
% A Monte Carlo analysis is also performed to assess controller robustness,
% and post-processing plots compare MIL, SIL, and PIL results.

% Before running the simulation, ensure that:
%   (1) The Simulink model file is correctly added to the MATLAB path
%   (2) The base workspace parameter structure is loaded
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% clear variables section
clear      % Remove all variables from workspace
close all  % Close all open figure windows
clc        % Clear the command window display

%% Load Satellite Model and Utility Functions
addpath("plantModel_2ndAssignment_prot_windows\")
load("plantModel_pointing_BaseWorkspace.mat")


%% NOMINAL SIMULATION
% intial conditions
q0 = [0.2226, 0.1019, -0.9678, 0.0589];
om0 = [0; 0; 0];
I = [0.058816667 0 0; 0 0.054000375 0; 0 0 0.011250375];

% Trial and error values for the gains
% kp = 4e-3;
% kd = 4e-3;
% wn = 4/eps/Ts;
% wn = wn*ones(3,1);

wn = 0.4;
eps = 0.9;

% control gains
kp = diag(I).*wn.^2;
kd = eps*wn.*diag(I);

% nominal simulation
out = sim("Richero280245_Assign2.slx");

% color code for the plots
c1 = [0.00 0.45 0.70];   % blue
c2 = [0.90 0.60 0.00];   % orange
c3 = [0.00 0.60 0.50];   % blu-verde

%%  Flag nominal plots
% Set flag_plot = 1 to enable the visualization of each results plot.
% When enabled, the simulation will generate and display the figures 
% corresponding to all test cases evaluated in the paper.

flag_nominal_plots = 1;

%% FOR THE MONTECARLO ANALYSIS SET TO 1
flag_montecarlo = 0;

%% Flag: set to 1 only if data of SIL and PIL are in the folder
% for these two flag both SIL and PIL matrices have to be loaded NOT
% present in the delivery folder as required
flag_SIL_MIL = 0;
flag_SIL_PIL = 0;



%% Nominal results
if flag_nominal_plots
% =========================
% FIGURE 1: pointing Error
% =========================
time = out.tout;
thErr = out.pointing_err;
time = time(~isnan(thErr));
thErr = thErr(~isnan(thErr));


fig = figure(1);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
hold on; grid on; box on;
ylim([0 0.25])
plot(time, thErr, 'LineWidth', 2, 'Color', c1)
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 16)
ylabel('Pointing Error [deg]', 'Interpreter','latex', 'FontSize', 16)


% ==========================
% FIGURE 2: Quaternion Error
% ==========================
qe = squeeze(out.quaternion_error);
qe = qe';

colors = lines(4); 
% Eliminate the initial elements related to Pointing
% Status different from three
qe_1 =  qe(1,:);
qe_1 = qe_1(~isnan(qe_1));
qe_2 =  qe(2,:);
qe_2 = qe_2(~isnan(qe_2));
qe_3 =  qe(3,:);
qe_3 = qe_3(~isnan(qe_3));
qe_4 =  qe(4,:);
qe_4 = qe_4(~isnan(qe_4));

fig = figure(2);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];


subplot(2,1,2)
hold on; grid on; box on;

h1 = plot(time, qe_1, 'LineWidth', 2, 'Color', colors(1,:));
h2 = plot(time, qe_2, 'LineWidth', 2, 'Color', colors(2,:));
h3 = plot(time, qe_3, 'LineWidth', 2, 'Color', colors(3,:));
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);

ylabel('Quaternion Error [-]', 'Interpreter', 'latex', 'FontSize', 18);
ylim([-2e-3, 2e-3])
legend([h1(1), h2(1), h3(1)], {'$q_{e1}$', '$q_{e2}$', '$q_{e3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

subplot(2,1,1)
hold on; grid on; box on;

h4 = plot(time, qe_4, 'LineWidth', 2, 'Color', colors(4,:));

xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
ylabel('Quaternion Error [-]', 'Interpreter', 'latex', 'FontSize', 18);
ylim([0.999999 1.0000004])
legend([h4(1)], {'$q_{e4}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

% ============================
% FIGURE 3: Angular Rate Error
% ============================
we = squeeze(out.w_error);

we_1 =  we(1,:);
we_2 =  we(2,:);
we_3 =  we(3,:);

time = out.tout;

fig = figure(3);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];


clf; hold on;

h1 = plot(time, we_1, 'Color', c1);
h2 = plot(time, we_2, 'Color', c2);
h3 = plot(time, we_3, 'Color', c3);

grid on; box on;
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
ylabel('Angular Rate Error [rad/s]', 'Interpreter','latex', 'FontSize', 18);
ylim([-2e-4, 2e-4])

legend([h1(1), h2(1), h3(1)],{'$\omega_{e1}$', '$\omega_{e2}$', '$\omega_{e3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');


% ============================
% FIGURE 5:  Control Torque
% ============================

u_RW = squeeze(out.u_RW);

time = out.tout;
u_RW_1 = u_RW(1,:);
u_RW_2 = u_RW(2,:);
u_RW_3 = u_RW(3,:);



fig = figure(5);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];

clf;
hold on
h1 = plot(time, u_RW_1, 'Color', c1);... 'LineWidth', 1.3);
h2 = plot(time, u_RW_2, 'Color', c2);... 'LineWidth', 1.3);
h3 = plot(time, u_RW_3, 'Color', c3);... 'LineWidth', 1.3);

grid on; box on;
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
ylabel('Control torque vector $[\mathrm{N \cdot m}]$', 'Interpreter','latex', 'FontSize', 18);
ylim([-3e-6, 3e-6])

legend([h1, h2, h3],{'$u_{RW1}$', ...
            '$u_{RW2}$', ...
            '$u_{RW3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

end

%% MONTE CARLO

if flag_montecarlo
% standard deviation for the initial angular rates 
sigma_deg = 10/3;
sigma_rad = deg2rad(sigma_deg);

% N samples
N = 200;
% Initialization of the N samples
om0_vec = normrnd(0, sigma_rad, [3,N]);
q0_vec = normrnd(0, 1, [4,N]);  
norm_q0_vec = vecnorm(q0_vec,2, 1)';
norm_q0_vec = norm_q0_vec*ones(1,4);
q0_vec = q0_vec ./ norm_q0_vec';
EulAng_vec = quat2eul(q0_vec')'*180/pi;

% empty array to be filled
thErr_sim = [];
% we_sim = [];
qe_1 = [];
qe_2 = [];
qe_3 = [];
qe_4 = [];

we_1 = [];
we_2 = [];
we_3 = [];
we_4 = [];

for i=1:N
    om0 = om0_vec(:,i);
    q0 = q0_vec(:,i)';
    out = sim("Richero280245_Assign2.slx");

    thErr = out.pointing_err;
    we = squeeze(out.w_error);
    qe = squeeze(out.quaternion_error)';

    qe_1 = [qe_1; qe(1,:)];
    qe_2 = [qe_2; qe(2,:)];
    qe_3 = [qe_3; qe(3,:)];
    qe_4 = [qe_4; qe(4,:)];

    we_1 = [we_1; we(1,:)];
    we_2 = [we_2; we(2,:)];
    we_3 = [we_3; we(3,:)];

    thErr_sim = [thErr_sim, thErr];

end

% om0_vec is [3 x N], each column is one sample of omega0


% omega distribution
fig = figure(6);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];

componentsx = {'$\omega_x\,[\mathrm{rad/s}]$', ...
               '$\omega_y\,[\mathrm{rad/s}]$', ...
               '$\omega_z\,[\mathrm{rad/s}]$'};
components = {'$\omega_x$', ...
               '$\omega_y$', ...
               '$\omega_z$'};
componentsy = {'$\%$','$\%$','$\%$'};

for i = 1:3
    subplot(3,1,i)
    hold on
    grid on
    box on

    % ---- Histogram ----
    h = histogram(om0_vec(i,:), N, ...
        'Normalization','percentage', ...
        'FaceColor','r', ...
        'FaceAlpha',0.8, ...
        'EdgeColor','k', ...
        'EdgeAlpha',0.1);

    % ---- Gaussian parameters ----
    mu    = 0;
    sigma = sigma_rad;

    % Support for PDF
    x = linspace(min(om0_vec(i,:)), max(om0_vec(i,:)), 500);

    % Gaussian PDF scaled to percentage histogram
    binWidth = h.BinWidth;
    y = 100 * binWidth * (1/(sigma*sqrt(2*pi))) * exp(-(x-mu).^2/(2*sigma^2));

    % ---- Gaussian plot ----
    h1 = plot(x, y, 'b--', 'LineWidth', 2);

    % ---- Labels ----
    xlabel(componentsx{i}, 'Interpreter','latex', 'FontSize',18)
    ylabel(componentsy{i}, 'Interpreter','latex', 'FontSize',18)
    legend([h1, h], {'Gaussian PDF', strcat(components{i}, ' distribution')}, 'Interpreter','latex', 'FontSize', 16)
end



% Euler Angles distribution
fig = figure(7);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];

componentsx = {'$\alpha$ [deg]','$\beta$ [deg]','$\gamma$ [deg]'};
components = {'$\alpha$','$\beta$','$\gamma$'};
componentsy = {'$\%$','$\%$','$\%$'};

for i = 1:3
    subplot(3,1,i);
    grid on
    box on
    h = histogram(EulAng_vec(i,:), N, ...
        'Normalization','percentage',...
        'FaceColor','b','FaceAlpha', 0.7,...
        'EdgeColor','k', 'EdgeAlpha',0.1); 
    grid on;
    xlabel(componentsx{i}, 'Interpreter','latex', 'FontSize',18);
    ylabel(componentsy{i}, 'Interpreter','latex', 'FontSize',18);

    legend([h], {strcat(components{i}, ' distribution')}, 'Interpreter','latex', 'FontSize', 16)
    % % ylabel('Count','Interpreter','latex', 'FontSize',16);
    % title(['Distribution of ', componentst{i}], 'Interpreter','latex','FontSize',16);
end

time = out.tout;
thErr_avg = mean(thErr_sim, 2);

% Pointing Error
fig = figure(8);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];

hold on
grid on
box on
h1 =plot(time,thErr_sim, 'LineWidth', 0.5, 'color',[0.7 0.7 0.7]);
h2 =plot(time,thErr_avg,'LineWidth' , 0.5,'color', 'r');
% h3 =yline(5, '--', 'color', 'k', 'LineWidth' , 1.5);
h4 =xline(500, '-.', 'Color','b', 'LineWidth' , 1.5);
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18)
ylabel('Pointing Error [deg]', 'Interpreter','latex', 'FontSize', 18)
ylim([0 0.2])
legend([h1(1) h2 h4], {'Pointing error','Mean', 'Max time limit'}, 'Interpreter','latex', 'FontSize', 16)


fig = figure(999);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];

hold on
grid on
box on
h1 =plot(time,thErr_sim, 'LineWidth', 0.8, 'color',[0.7 0.7 0.7]);
h2 =plot(time,thErr_avg,'LineWidth' , 0.8,'color', 'r');
h3 =yline(5, '--', 'color', 'k', 'LineWidth' , 1.5);
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18)
ylabel('Pointing Error [deg]', 'Interpreter','latex', 'FontSize', 18)
xlim([1 40])
legend([h1(1) h2 h3], {'Pointing error','Mean', 'Max allowable error'}, 'Interpreter','latex', 'FontSize', 16)

colors = lines(4); 

% Quaternion Error
fig = figure(9);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf;

subplot(2,1,2)
hold on; grid on; box on;

h1 = plot(time, qe_1, 'LineWidth', 0.8, 'Color', colors(1,:));
h2 = plot(time, qe_2, 'LineWidth', 0.8, 'Color', colors(2,:));
h3 = plot(time, qe_3, 'LineWidth', 0.8, 'Color', colors(3,:));
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);

ylabel('Quaternion Error [-]', 'Interpreter', 'latex', 'FontSize', 18);
ylim([-2.5e-3, 2.5e-3])
legend([h1(1), h2(1), h3(1)], {'$q_{e1}$', '$q_{e2}$', '$q_{e3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

subplot(2,1,1)
hold on; grid on; box on;

h4 = plot(time, qe_4, 'LineWidth', 0.8, 'Color', colors(4,:));

xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
ylabel('Quaternion Error [-]', 'Interpreter', 'latex', 'FontSize', 18);
ylim([0.9999993 1.0000001])
legend([h4(1)], {'$q_{e4}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');


% Angular Rate Error
fig = figure(10);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; hold on;
c1 = [0.00 0.45 0.70];   % blue
c2 = [0.90 0.60 0.00];   % orange

h1 = plot(time, we_1, 'LineWidth', 0.3, 'Color', c1);
h2 = plot(time, we_2, 'LineWidth', 0.3, 'Color', c2);
h3 = plot(time, we_3, 'LineWidth', 0.3, 'Color', 'c');

grid on; box on;
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
ylabel('Angular Rate Error [rad/s]', 'Interpreter','latex', 'FontSize', 18);
ylim([-2e-4, 2e-4])
legend([h1(1), h2(1), h3(1)],{'$\omega_{e1}$', '$\omega_{e2}$', '$\omega_{e3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');


fig = figure(11);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; hold on;
c1 = [0.00 0.45 0.70];   % blue
c2 = [0.90 0.60 0.00];   % orange

we_all   = {we_1, we_2, we_3};
lab_y    = {'$\omega_{e1}\,[\mathrm{rad/s}]$', ...
            '$\omega_{e2}\,[\mathrm{rad/s}]$', ...
            '$\omega_{e3}\,[\mathrm{rad/s}]$'};

subplot(3,1,1);
h1 = plot(time, we_1, 'Color', c1, 'LineWidth', 0.8);
grid on; box on;
ylim([-2e-4, 2e-4])
ylabel(lab_y{1}, 'Interpreter','latex', 'FontSize', 18);
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
legend([h1(1)],{'$\omega_{e1}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

subplot(3,1,2);
h2 = plot(time, we_2, 'Color', c2, 'LineWidth', 0.8);
grid on; box on;
ylim([-2e-4, 2e-4])
ylabel(lab_y{2}, 'Interpreter','latex', 'FontSize', 18);
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
legend([h2(1)],{'$\omega_{e2}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');

subplot(3,1,3);
h3 = plot(time, we_3, 'Color', c3, 'LineWidth', 0.8);
grid on; box on;
ylim([-2e-4, 2e-4])
ylabel(lab_y{2}, 'Interpreter','latex', 'FontSize', 18);
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
legend([h3(1)],{'$\omega_{e3}$'}, ...
       'Interpreter','latex', 'FontSize', 16, 'Location','best');
end




%% SIL-MIL comparison

if flag_SIL_MIL
% load data updated from the Data inspector
load('MIL_uRW.mat','data');  
MIL_uRW = squeeze(data.Data);
clear data
load('SIL_uRW.mat','data');
SIL_uRW = squeeze(data.Data);

time = data.Time;


% colors 
cMIL = [0.00 0.45 0.70];   % blue
cSIL = [0.85 0.33 0.10];   % orange

% =========================
% FIGURE 1: MIL vs SIL
% =========================
fig = figure(12);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; 

for i = 1:3
    subplot(3,1,i); hold on; grid on; box on

    hMIL = plot(time, MIL_uRW(i,:), 'Color', cMIL, 'LineWidth', 1.3);
    hSIL = plot(time, SIL_uRW(i,:), 'Color', cSIL, 'LineWidth', 1.3, 'LineStyle','--');
    ylim([-2.5e-6, 2.5e-6])

    ylabel(sprintf('$u_{RW%d}\\,[\\mathrm{N\\cdot m}]$',i), ...
        'Interpreter','latex', 'FontSize', 16);

    legend([hMIL hSIL], ...
        {sprintf('$u_{RW%d}$ MIL',i), sprintf('$u_{RW%d}$ SIL',i)}, ...
        'Interpreter','latex', 'FontSize', 13, 'Location','best');

    if i ~= 3
        set(gca,'XTickLabel',[]);
    else
        xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
    end
end

% =========================
% FIGURE 2: absolute difference
% =========================
fig = figure(13);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; 

du = abs(MIL_uRW - SIL_uRW);

[du1_max, i1_max] = max(du(1,:));
[du2_max, i2_max] = max(du(2,:));
[du3_max, i3_max] = max(du(3,:));

du_max = [du1_max, du2_max, du3_max];
i_max = [i1_max i2_max i3_max];


for i = 1:3
    subplot(3,1,i); hold on; grid on; box on

    hD = plot(time, du(i,:),'color', [0.5 0.5 0.5], 'LineWidth', 0.5);
    h2 = yline(1.5*du_max(i), '--r', 'LineWidth', 1.5);
    h3 = scatter(time(i_max(i)), du_max(i), '*b');

    ylabel(sprintf('$|\\Delta u_{RW%d}|\\,[\\mathrm{N\\cdot m}]$',i), ...
        'Interpreter','latex', 'FontSize', 16);

    legend([hD h2 h3], {sprintf('$|u_{RW%d}^{^{^{MIL}}} - u_{RW%d}^{^{^{SIL}}}|$',i,i), 'tolerance', 'Max'}, ...
        'Interpreter','latex', 'FontSize', 13, 'Location','best');

    if i ~= 3
        set(gca,'XTickLabel',[]);
    else
        xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
    end
end

end

%% SIL-PIL comparison

if flag_SIL_PIL
% load data updated from the Data inspector
load('PIL_uRW.mat','data');  
PIL_uRW = squeeze(data.Data);
load('SIL_uRW.mat','data');  
SIL_uRW = squeeze(data.Data);

time = data.Time;

% color code
cPIL = 'k';  
cSIL = [0.85 0.33 0.10];   % orange

% =========================
% FIGURE 1: MIL vs SIL
% =========================
fig = figure(14);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; 

for i = 1:3
    subplot(3,1,i); hold on; grid on; box on

    hPIL = plot(time, PIL_uRW(i,:), 'Color', cPIL, 'LineWidth', 1.3);
    hSIL = plot(time, SIL_uRW(i,:), 'Color', cSIL, 'LineWidth', 1.3, 'LineStyle','--');
    
    ylim([-2.5e-6, 2.5e-6])

    ylabel(sprintf('$u_{RW%d}\\,[\\mathrm{N\\cdot m}]$',i), ...
        'Interpreter','latex', 'FontSize', 16);

    legend([hPIL hSIL], ...
        {sprintf('$u_{RW%d}$ PIL',i), sprintf('$u_{RW%d}$ SIL',i)}, ...
        'Interpreter','latex', 'FontSize', 13, 'Location','best');

    if i ~= 3
        set(gca,'XTickLabel',[]);
    else
        xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
    end
end

% =========================
% FIGURE 2: absolute difference
% =========================
fig = figure(15);
fig.Units = 'pixels';
fig.Position = [0 0 1800 900];
clf; 

du = abs(PIL_uRW - SIL_uRW);

for i = 1:3
    subplot(3,1,i); hold on; grid on; box on

    hD = plot(time, du(i,:),'color', [0.3 0.3 0.3], 'LineWidth', 0.5);
    h2 = yline(1.5*du_max(i), '--r');

    ylabel(sprintf('$|\\Delta u_{RW%d}|\\,[\\mathrm{N\\cdot m}]$',i), ...
        'Interpreter','latex', 'FontSize', 16);

    legend([hD, h2], {sprintf('$|u_{RW%d}^{^{^{PIL}}} - u_{RW%d}^{^{^{SIL}}}|$',i,i), 'tolerance'}, ...
        'Interpreter','latex', 'FontSize', 13, 'Location','best');

    ylim([0, 6e-15])

    if i ~= 3
        set(gca,'XTickLabel',[]);
    else
        xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 18);
    end
end


end