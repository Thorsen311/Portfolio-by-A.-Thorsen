clc; clearvars -except ; close all;
set(0,'DefaultFigureWindowStyle','docked') %Normal or docked
set(0,'DefaultFigureVisible','on')  %If you want to see the plot switch "off" with "on"
set(groot, 'defaultAxesTickLabelInterpreter','latex')
set(groot, 'defaultLegendInterpreter','latex')
set(groot,'defaulttextinterpreter','latex');  
set(groot,'DefaultAxesTitle', 'latex')
set(0, 'DefaultAxesFontSize', 13);
get(0,'Factory');
set(0,'defaultfigurecolor',[1 1 1])

%% CODE DESCRIPTION
% Hi! This code is created by Anton Thorsen as part of the course; Vehicle
% Dynamics and Stability. This particular code (HW7_VDAS.m) is part 
% of Homework 7, and is used to solve subtasks 7.1 and 7.2. To solve task
% 7.1, lines 154 and 155 must be commented, and decel set to be 0.39 to
% ensure no lockups. To solve task 7.2, lines 154 and 155 must NOT be
% commented.
% The stop condition is controlled in the function odeRK4 line 297

%% Defining
% 1998 Honda Civic Vehicle No. 452: https://www.eng.auburn.edu/~dmbevly/mech4420/vehicle_params.pdf

% Time and natural constants
dt = 0.01;              % seconds - set time step
tspan = 0:dt:20;        % Set time span
g = 9.8;                % Acceleration due to gravity (m/s^2)


% Car parameters    
m = 1143;               % kg
a = 1.038;              % m
L = 2.621;              % m
b = L-a;                % m 
Izz = 1785;             % kg*m^2
C_f = 50990.278;        % N/rad  
C_r = 50990.278;        % N/rad 
p_bar = 0.5;            % Weight distribution (front/rear)
H2 = 0.513;             % Height of center of mass
T = 1.695;              % m width of car
W = m*g;                % Weight of the car

X_initial = -20;        % m Initial condition
Y_shift = -3;
psi_initial = atan(Y_shift/X_initial);

%The variables to be optimized:
%Found solutions: 
%ui = 17.22222; decel = 0.42222; X_brake = 26.66667; delta_f = -1.66667*pi/180; %t=4.51
%u_i = 17.7778; decel = 0.44444; X_brake = 26.22222; delta_f = -1.72222*pi/180: %t=4.39
ui = 17.7778; decel = 0.44444; X_brake = 26.22222; delta_f = -1.72222*pi/180;

%Roll
phi_bar = -4*pi/180;    % rad/g this is negative ROLL GAIN
freq_R = 1.5;           % Hz for normal car 
K = ((-W*H2)/phi_bar) + W*H2;
Ixx = (K-W*H2)/((freq_R*2*pi)^2);
c_roll = 0.2*2*sqrt(Ixx*(K-W*H2));
delta_r = 0*pi/180;     % roll steer in rad/g;

%Braking and pitch
BLIMIT = 1787.88;       % 1787.88 is result from HW 6
Q_brak = 2.48;          % braking ratio result from HW6
freq_P = 1;             % Hz
theta_ss = (-0.1);      % rad/g Steady state pitch
K_bar = (-W*H2) / (theta_ss);
Iyy = K_bar / ((2*pi*freq_P)^2);
c_pitch = 0.2*2*sqrt(Iyy*(K_bar));
K_dbar =((W*H2)/L)/(theta_ss);

%Storing all properties in 'car'
car = struct('tspan',tspan, 'm', m, 'a', a, 'L', L, 'b', b, 'Izz', Izz, 'C_f', C_f, 'C_r', C_r, 'g', g, ...
    'delta_f',delta_f,'p_bar',p_bar, 'H2',H2,'W',W,'T',T,'phi_bar',phi_bar,'K',K,'Ixx',Ixx,'c_roll',c_roll, 'delta_r', ...
    delta_r,'BLIMIT',BLIMIT,'Q_brak', Q_brak ,'theta_ss',theta_ss,'K_bar',K_bar,'Iyy',Iyy,'c_pitch',c_pitch, 'K_dbar',K_dbar, ...
    'X_brake',X_brake,'decel',decel);

% Initial conditions
r0 = 0; v0 = 0; glob_X0 = X_initial; glob_Y0 = 0; psi0 = psi_initial; phi0 = 0; phi_dot0 = 0; u0 = ui; theta0 = 0; theta_dot0 = 0;  %Velocity at beginning is given

% Initial conditions with additional state variables
X0 = [r0; v0; glob_X0; glob_Y0; psi0; phi0; phi_dot0; u0; theta0; theta_dot0];


%% Calculations
[X, a_y, a_x, N_all, Fx_all, BF, delta_f, LU, Fy_all] = odeRK4(@car_modelNL, X0, tspan, car);
tspan(length(X)+1:end) = [];    % Removing all points after vehicle is stopped

psi = X(end,5); glob_X = X(end,3); glob_Y = X(end, 4);
[~,~,~,~, F_wx, F_wy] = fric_coeff(glob_X, psi, car, glob_Y);
F_wy_RR = -F_wy(end,2); %Minus since the y-axis is flipped
F_wy_FL = -F_wy(end,4);
velo_end = sqrt(X(end,8)^2 + X(end,2)^2);
%Possible solution: velo_i = 15; decel = 0.4; X_brake = 30; delta_f =
%-1.7*pi/180
t = tspan(length(X));
fprintf('Solution time %.5f \n',t)
[f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12] = plot_all(tspan, X, a_y, a_x, N_all, Fx_all, BF, delta_f, LU, Fy_all);
f = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12];
subplotRows = 3;
subplotCols = 4;
subplot_temp(f,subplotRows,subplotCols)

%Patch plot:
[xdata, ydata, zdata]=car_vis(a, b, T, X(:,3), X(:,4), X(:,5), 30); %The last input changes number of cars - low number more cars
figure(99)
    hold on
    p = patch(xdata,-ydata,zdata,'b');
    title('Patched Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    ylim([-10 10])
    xlim([-22 52])
    axis equal
    plot(50, 0,'ro','MarkerSize',10,'MarkerFaceColor','r')
    plot(0, -3,'ro','MarkerSize',10,'MarkerFaceColor','r')
    xline(0)
    yline(0)
    legend('Patched Trajectory','','','')
    grid on
    box on;
    yticks([-30, -20, -10, 0, 10, 20]);
    yticklabels({'30', '20', '10', '0', '-10', '-20'});
    hold off

%% Computing functions
function [Xdot, a_y, a_x, N_all, Fx_all, BF, delta_f, LU, Fy_all] = car_modelNL(X, t, car) %n is the iteration number
    tspan = car.tspan; m = car.m; a = car.a; L = car.L; b = car.b; Izz = car.Izz; C_f = car.C_f; C_r = car.C_r; g = car.g;
    delta_f = car.delta_f; p_bar = car.p_bar; H2 = car.H2; W = car.W; T = car.T; phi_bar = car.phi_bar; K = car.K; Ixx = car.Ixx; c_roll = car.c_roll;
    delta_r = car.delta_r;BLIMIT = car.BLIMIT; Q_brak = car.Q_brak; theta_ss = car.theta_ss; Iyy = car.Iyy; K_bar = car.K_bar; H2 = car.H2; c_pitch = car.c_pitch; 
    K_dbar = car.K_dbar; X_brake = car.X_brake; decel = car.decel;
    
    r = X(1);
    v = X(2);
    glob_X = X(3);
    glob_Y = X(4);
    psi = X(5);
    phi = X(6);
    phi_dot = X(7);
    u = X(8);               % New variables for longitudinal speed, pitch and pitch rate
    theta = X(9);
    theta_dot = X(10);
    
    Q = delta_r/phi_bar;    % delta_r is 0, so this is 0

    if glob_X < 0 
        delta_f = 0;
        Q = 0;
    else
        delta_f = delta_f;
        Q = Q;
    end
    
    %Static loads
    N_f = (W * b) / L;
    N_r = (W * a) / L;

    %If we are not braking
    BF_f = 0; BF_r = 0;  DeltaN_pitch = 0; %When not braking there is no brake force, and no pitch.
    
    % Normal loads - the only difference in normal loads w/o breaking are left-to-right because of roll
    DeltaN_roll = 2 / T * (-K * phi - c_roll * phi_dot); 
    DeltaN_f_roll = p_bar * DeltaN_roll;
    DeltaN_r_roll = (1 - p_bar) * DeltaN_roll;

    %If we are braking:
    if glob_X > X_brake %Break when ~steady state, at t > t_brake
        %decel = 0.39;
        [~, BF_f, BF_r, ~, ~, ~, ~, ~] = brake_no_trailer(car, decel);  %Calculate forces when braking
        DeltaN_pitch = K_dbar*theta;
        %BF_f = 10^6; %To lock up front wheels
        %BF_r = 10^6; %To lock up rear wheels
    end
    
    BF = [BF_f, BF_r];
    % Brake forces
    SUM_BF = BF_f + BF_r; %[UNUSED]
    
    %Normal loads
    N_fl = N_f / 2 + DeltaN_f_roll / 2 + DeltaN_pitch / 2;
    N_fr = N_f / 2 - DeltaN_f_roll / 2 + DeltaN_pitch / 2; 
    N_rl = N_r / 2 + DeltaN_r_roll / 2 - DeltaN_pitch / 2; 
    N_rr = N_r / 2 - DeltaN_r_roll / 2 - DeltaN_pitch / 2;
    N_all = [N_fl, N_fr, N_rl, N_rr];

    % Slip angles and lateral forces
    alpha_fL = atan2( v + a * r, u + T*r/2) - delta_f;
    alpha_fR = atan2( v + a * r, u - T*r/2) - delta_f;
    alpha_rL = atan2( v - b * r, u + T*r/2) - Q*phi;
    alpha_rR = atan2( v - b * r, u - T*r/2) - Q*phi;

    [mu_fL, mu_fR, mu_rL, mu_rR, ~, ~] = fric_coeff(glob_X, psi, car, glob_Y);

    [Fx_FL_tire, Fy_FL_tire, LU(1)] = NLTire(C_f, alpha_fL, mu_fL, N_fl, BF_f/2);   %Tiremodel is for 1 wheel, so divide BF with 2
    [Fx_FR_tire, Fy_FR_tire, LU(2)] = NLTire(C_f, alpha_fR, mu_fR, N_fr, BF_f/2);  
    [Fx_RL_tire, Fy_RL_tire, LU(3)] = NLTire(C_r, alpha_rL, mu_rL, N_rl, BF_r/2, BLIMIT);
    [Fx_RR_tire, Fy_RR_tire, LU(4)] = NLTire(C_r, alpha_rR, mu_rR, N_rr, BF_r/2, BLIMIT);

    Fx_FL = Fx_FL_tire*cos(delta_f) - Fy_FL_tire*sin(delta_f);
    Fy_FL = Fx_FL_tire*sin(delta_f) + Fy_FL_tire*cos(delta_f);
    Fx_FR = Fx_FR_tire*cos(delta_f) - Fy_FR_tire*sin(delta_f);
    Fy_FR = Fx_FR_tire*sin(delta_f) + Fy_FR_tire*cos(delta_f);
    Fx_RL = Fx_RL_tire;
    Fy_RL = Fy_RL_tire;
    Fx_RR = Fx_RR_tire;
    Fy_RR = Fy_RR_tire;
    Fx_all = [Fx_FL, Fx_FR, Fx_RL, Fx_RR];
    Fy_all = [Fy_FL, Fy_FR, Fy_RL, Fy_RR];

    % Calculate SUMs
    SUM_Ff_lat = Fy_FL + Fy_FR;
    SUM_Fr_lat = Fy_RL + Fy_RR;
    SUM_lat = SUM_Ff_lat + SUM_Fr_lat;
    
    SUM_Ff_long = Fx_FL + Fx_FR;
    SUM_Fr_long = Fx_RL + Fx_RR;
    SUM_long = SUM_Ff_long + SUM_Fr_long;

    % Calculate dummy variables
    phi_ddot = ((-(SUM_lat)*H2) - ((K - W * H2) * phi) - c_roll * phi_dot) / Ixx;
    theta_ddot = (((SUM_long)*H2) - (K_bar * theta) - c_pitch * theta_dot) / Iyy;  %Han sagde det skulle være SUM_long og ingen minus
    
    % Yaw acceleration and lateral acceleration
    r_dot = (SUM_Ff_lat * a - SUM_Fr_lat * b + (Fx_FL - Fx_FR + Fx_RL - Fx_RR)*T/2) / Izz;  
    v_dot = (SUM_lat / m) - (u * r) - H2 * phi_ddot;
    u_dot = (SUM_long / m) + (v * r); % Should I add a phi_ddot term?
    glob_X_dot = u * cos(psi) - v * sin(psi);
    glob_Y_dot = u * sin(psi) + v * cos(psi);
    psi_dot = r;
    
    % Extracting acceleration
    a_y = SUM_lat / m;
    a_x = SUM_long / m;

    % State derivatives
    Xdot(1, 1) = r_dot;
    Xdot(2, 1) = v_dot;
    Xdot(3, 1) = glob_X_dot;
    Xdot(4, 1) = glob_Y_dot;
    Xdot(5, 1) = psi_dot;
    Xdot(6, 1) = phi_dot;
    Xdot(7, 1) = phi_ddot;
    
    % Derivatives for new variables
    Xdot(8, 1) = u_dot;
    Xdot(9, 1) = theta_dot;
    Xdot(10, 1) = theta_ddot;
end

function [decel, BF_f, BF_r, N_f, N_r, mu_f, mu_r, eta] = brake_no_trailer(car, decel)
m = car.m; a = car.a; L = car.L; b = car.b; g=car.g; H2 = car.H2; W = car.W; BLIMIT = car.BLIMIT; Q_brak = car.Q_brak; K_dbar = car.K_dbar;

Nf_static = W*b/L;
Nr_static = W*a/L;

% Calculate rear brake force
BF_r = W * decel / (1 + Q_brak);
BF_f = Q_brak*BF_r;

if BF_r > BLIMIT                  
  BF_r = BLIMIT;
  if Q_brak ~= 0 %Check if we have rear-brakes only-case
    BF_f = W * decel - BF_r;
  end
end

% Calculate normal loads
N_f = Nf_static + W * (decel) * H2 / L; 
N_r = Nr_static - W * (decel) * H2 / L;

% Calculate coefficients of friction
mu_f = BF_f / N_f;
mu_r = BF_r / N_r;

% Determine effective coefficient of friction
mu = mu_f;
if mu_r > mu
    mu = mu_r;
end

% Calculate brake efficiency
eta = decel / mu;
end

function [Fx_tire, Fy_tire, LU] = NLTire(C_alpha, alpha, mu, N, BF, varargin)
if ~isempty(varargin)
    BLIMIT = varargin{1};
end
LU = 0;
Fx = -BF;
Fy = -C_alpha*alpha;
    if abs(Fy) > (mu*N)/2
       Fy = -mu*N*sign(alpha) * (1 - (mu*N)/(4*C_alpha*abs(tan(alpha))));
       LU = 2;
    end  
% Check for lockup
F_check = -mu*N*cos(alpha);
    if abs(F_check) < BF
        Fy = -mu*N*sin(alpha); %Case II
        Fx = F_check;
        LU = 0.5;
    elseif (mu*N)^2 < (BF^2 + Fy^2) 
        Fy = -sign(alpha)*sqrt((mu*N)^2 - (BF)^2); %Case III
        Fx = -BF;
        LU = 1;
    end
if ~isempty(varargin)
    if BLIMIT == 2*BF
        LU = 1.5;
    end
end
Fy_tire = Fy;
Fx_tire = Fx;
end

function [Y, a_y, a_x, N_all, Fx_all, BF, delta_f, LU, Fy_all] = odeRK4(dYdt, Y0, t, car)
% get size of problem
ne = length(Y0); % no. equations
nt = length(t);  % timesteps
h  = t(2)-t(1);  % step size

% initialize output 
Y = zeros(nt,ne);  %(tom matrix)
Y(1,:) = Y0(:)'; % store results row-wise
% marching forward through time
    for n = 1:nt-1
        Yn = Y(n,:)';    %Take the first row of Y and turn it into a column vector
        [~, a_y(n+1,:),a_x(n+1,:), N_all(n+1,:), Fx_all(n+1,:), BF(n+1,:), delta_f(n+1,:), LU(n+1,:), Fy_all(n+1,:)] = dYdt(Yn, t(n), car);  %Extraction lateral accelerations
        k1 = dYdt(Yn,           t(n), car);  
        k2 = dYdt(Yn+(h/2)*k1,  t(n)+(h/2), car);
        k3 = dYdt(Yn+(h/2)*k2,  t(n)+(h/2), car);
        k4 = dYdt(Yn+h*k3,      t(n)+h, car);
        Y(n+1,:) = Yn + h/6*k1 + h/3*(k2+k3) + h/6*k4; 
        if Y(n,3) >= 50  %If x-position is above 50 meters stop
            Y(n+2:end,:) = [];
            return
        end

    end
end


function [mu_fL, mu_fR, mu_rL, mu_rR, F_wx, F_wy] = fric_coeff(X, psi, car, Y)
T = car.T; a = car.a; b=car.b;
% Distance between CP and a front wheel
r_a=(a^2+(T/2)^2)^0.5;
% Distance between CP and a rear wheel
r_b=(b^2+(T/2)^2)^0.5;
% Angle between the x-axis of the car and the distance vector to the front
% wheel
v_a=atan(T/(2*a));
% Angle between the x-axis of the car and the distance vector to the rear
% wheel
v_b=atan(T/(2*b));

% Orientation equation
FR_wx=X+r_a*(cos(v_a)*cos(psi)-sin(v_a)*sin(psi));
RR_wx=X+r_b*(-cos(v_b)*cos(psi)-sin(v_b)*sin(psi));
RL_wx=X+r_b*(-cos(v_b)*cos(psi)+sin(v_b)*sin(psi));
FL_wx=X+r_a*(cos(v_a)*cos(psi)+sin(v_a)*sin(psi));

FR_wy=Y+r_a*(cos(v_a)*sin(psi)+sin(v_a)*cos(psi));
RR_wy=Y+r_b*(-cos(v_b)*sin(psi)+sin(v_b)*cos(psi));
RL_wy=Y+r_b*(-cos(v_b)*sin(psi)-sin(v_b)*cos(psi));
FL_wy=Y+r_a*(cos(v_a)*sin(psi)-sin(v_a)*cos(psi));
F_wx = [FR_wx, RR_wx, RL_wx, FL_wx];
F_wy = [FR_wy, RR_wy, RL_wy, FL_wy];


% Set friction coefficients based on wheel locations
if  (FL_wx < 0) %If the front left positive then i
    mu_fL = 0.3;  
else 
    mu_fL = 0.9;
end

if  (FR_wx < 0)
    mu_fR = 0.3;
else 
    mu_fR = 0.9;
end

if  (RL_wx < 0)
    mu_rL = 0.3;
else 
    mu_rL = 0.9;
end

if  (RR_wx < 0)
    mu_rR = 0.3;
else 
    mu_rR = 0.9;
end

% mu_fL(FL_w >= 0) = 0.8;  % Positive FR_w
% mu_fL(FL_w < 0) = 0.1;  % Negative FR_w
% 
% mu_fR(FR_w >= 0) = 0.8;  % Positive FR_w
% mu_fR(FR_w < 0) = 0.1;  % Negative FR_w
% 
% mu_rL(RL_w >= 0) = 0.8;  % Positive RL_w
% mu_rL(RL_w < 0) = 0.1;  % Negative RL_w
% 
% mu_rR(RR_w >= 0) = 0.8;  % Positive RR_w
% mu_rR(RR_w < 0) = 0.1;  % Negative R
end



%% Plotting functions
function [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12] = plot_all(tspan, X, a_f, a_x, N_all, Fx_all, BF, delta_f, LU, Fy_all)
g = 9.8;
size = 1.5;
color = num2str(get(gca,'colororder'));

%Plot longitudinal and lateral acceleration
f1 = figure(1);
    hold on
    plot(tspan, a_x(:,1)./g,'LineWidth',size);
    plot(tspan, a_f(:,1)./g,'LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    xlim([0, tspan(end)])
    title('Long. and lat. accerelation vs Time');
    legend('Long.','Lat', 'Location', 'Best')
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    box on;

%Plot velocity
f2 = figure(2);
    hold on
    plot(tspan, sqrt(X(:,8).^2 + X(:,2).^2),'LineWidth',1.5);
    %legend(,'Location','southeast')
    hold off
    title('Velocity, $\sqrt{(u^2 + v^2)}$, vs Time');
    xlabel('Time (s)');
    xlim([0, tspan(end)])
    ylabel('Velocity (m/s)');
    box on;

% Plot sideslip angle
beta = atan2(X(:,2),X(:,8)) .* 180/pi;
%beta = atan(X(:,2)./X(:,8)) .* 180/pi;
f3 = figure(3);
    hold on
    plot(tspan, beta,'LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    xlim([0, tspan(end)])
    title('Sideslip angle vs Time');
    xlabel('Time (s)');
    ylabel('Sideslip angle (deg)');
    box on;

% Plot yaw rate
f4 = figure(4);
    hold on
    plot(tspan, X(:,1),'LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    xlim([0, tspan(end)])
    title('Yaw Rate vs Time');
    xlabel('Time (s)');
    ylabel('Yaw Rate (rad/s)');
    box on;


%Longitudinal Forces
f5 = figure(5);
    plot(tspan, Fx_all(:, 1),'Color',color(1,:), 'LineWidth', 2);
    hold on;
    plot(tspan, Fx_all(:, 2),'Color', color(2,:), 'LineWidth', 2,'LineStyle','--');
    plot(tspan, Fx_all(:, 3),'Color', color(3,:), 'LineWidth', 2);
    plot(tspan, Fx_all(:, 4),'Color', color(4,:), 'LineWidth', 2,'LineStyle','--');
    hold off
    xlim([0, tspan(end)])
    xlabel('Time (s)');
    ylabel('Longitudinal forces (N)');
    legend('Front left','Front right','Rear left','Rear right', 'Location', 'Best');
    title('Longitudinal forces on front and back');
    grid on;
    box on;

%Normal Forces
% Plotting Normal loads for each tire
f6 = figure(6);
    plot(tspan, N_all(:, 1),'Color',color(1,:), 'LineWidth', 2);
    hold on;
    plot(tspan, N_all(:, 2),'Color', color(2,:), 'LineWidth', 2, 'LineStyle','--');
    plot(tspan, N_all(:, 3),'Color', color(3,:), 'LineWidth', 2);
    plot(tspan, N_all(:, 4),'Color', color(4,:), 'LineWidth', 2, 'LineStyle','--');
    hold off
    xlim([0, tspan(end)])
    xlabel('Time (s)');
    ylabel('Normal Load (N)');
    legend('Front left', 'Front right', 'Rear left', 'Rear right', 'Location', 'Best');
    title('Normal Loads on Each Tire');
    grid on;
    box on;

%Plot trajectory
f7 = figure(7);
    hold on
    plot(X(:,3), -X(:,4),'LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    title('Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal
    box on;

%Plotting lockups
f8 = figure(8);
    hold on
    plot(tspan, LU(:,1),'LineWidth',size);
    plot(tspan, LU(:,2),'--','LineWidth',size);
    plot(tspan, LU(:,3),'-.','LineWidth',size);
    plot(tspan, LU(:,4),':','LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    title('Lockups for each wheel');
    legend('Front left','Front right','Rear left','Rear right','Location','northwest')
    xlabel('Time (s)');
    ylabel('Lockup (LU)');
    yticks([0, 0.5, 1, 1.5, 2])
    yticklabels({'No LU', 'Case II (LU)', 'Case III','BLIMIT reached','Nonlinear'})
    ylim([-0.1 2.1])
    box on;
 
%Pitch
f9 = figure(9);
    hold on
    plot(tspan, X(:,9)*180/pi,'LineWidth',size);
    plot(tspan, X(:,6)*180/pi,'LineWidth',size);
    %legend(,'Location','southeast')
    hold off
    title('Pitch and Roll');
    legend('Pitch','Roll','Location','Best')
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    box on;

f10 = figure(10);
    hold on
    plot(tspan, BF(:,1),'LineWidth',size);
    plot(tspan, BF(:,2),'LineWidth',size);
    legend('Brake force front','Brake force rear','Location','southeast')
    hold off
    title('Brake forces');
    xlabel('Time (s)');
    ylabel('Force (N)');
    box on;

f11 = figure(11);
    hold on
    plot(tspan, delta_f*180/pi,'LineWidth',size);
    hold off
    title('Steer angle');
    xlabel('Time (s)');
    ylabel('Steer angle (deg)');
    box on;

%Vertical Forces
f12 = figure(12);
    plot(tspan, Fy_all(:, 1),'Color',color(1,:), 'LineWidth', 2);
    hold on;
    plot(tspan, Fy_all(:, 2),'Color', color(2,:), 'LineWidth', 2,'LineStyle','--');
    plot(tspan, Fy_all(:, 3),'Color', color(3,:), 'LineWidth', 2);
    plot(tspan, Fy_all(:, 4),'Color', color(4,:), 'LineWidth', 2,'LineStyle','--');
    hold off
    xlim([0, tspan(end)])
    xlabel('Time (s)');
    ylabel('Vertical forces (N)');
    legend('Front left','Front right','Rear left','Rear right', 'Location', 'Best');
    title('Vertical forces on front and back');
    grid on;
    box on;
end

function [xdata, ydata, zdata]=car_vis(a, b, B_v, X, Y, psi, vis_jump)

% Function that finds the corner points of a car for visualising the
% orientation of the car on a global plot.
% The inputs for function is:
% car_vis(distance from front wheels to CG , distance from back wheels to
% CG , Width of car , Global X-coordinate , Global Y-coordinate , Angle of
% the car, Jump in visualisation loop [1 = highest resolution])
% In the main file where the plots are created insert use the patch
% operation to visualise the car:
% p = patch(xdata,ydata,zdata,'b');

% Distance between CP and a front wheel
r_a=(a^2+(B_v/2)^2)^0.5;
% Distance between CP and a rear wheel
r_b=(b^2+(B_v/2)^2)^0.5;
% Angle between the x-axis of the car and the distance vector to the front
% wheel
v_a=atan(B_v/(2*a));
% Angle between the x-axis of the car and the distance vector to the rear
% wheel
v_b=atan(B_v/(2*b));

car_r_n=linspace(1,length(X),length(X)/vis_jump);
car_p=zeros(length(car_r_n),8);

% Orientation equation
for i=1:length(car_r_n)
    j=round(car_r_n(i));
    car_p(i,1)=X(j)+r_a*(cos(v_a)*cos(psi(j))-sin(v_a)*sin(psi(j))); %Front right
    car_p(i,2)=Y(j)+r_a*(cos(v_a)*sin(psi(j))+sin(v_a)*cos(psi(j))); %Front right
    car_p(i,3)=X(j)+r_b*(-cos(v_b)*cos(psi(j))-sin(v_b)*sin(psi(j))); %Rear right
    car_p(i,4)=Y(j)+r_b*(-cos(v_b)*sin(psi(j))+sin(v_b)*cos(psi(j))); %Rear right
    car_p(i,5)=X(j)+r_b*(-cos(v_b)*cos(psi(j))+sin(v_b)*sin(psi(j))); %Rear left
    car_p(i,6)=Y(j)+r_b*(-cos(v_b)*sin(psi(j))-sin(v_b)*cos(psi(j))); %Rear left
    car_p(i,7)=X(j)+r_a*(cos(v_a)*cos(psi(j))+sin(v_a)*sin(psi(j)));  %Front left
    car_p(i,8)=Y(j)+r_a*(cos(v_a)*sin(psi(j))-sin(v_a)*cos(psi(j)));  %Front left
end


xdata=[car_p(:,1)';car_p(:,3)';car_p(:,5)';car_p(:,7)'];
ydata=[car_p(:,2)';car_p(:,4)';car_p(:,6)';car_p(:,8)'];
zdata=ones(size(xdata));
end

function subplot_temp(individualFigures,subplotRows,subplotCols)
    subplotFig = figure;
    % Copy data from individual figures to subplots
    for i = 1:length(individualFigures)
        % Get the data from the individual plot
        srcFigure = individualFigures(i);
        srcAxis = gca(srcFigure);
        xData = get(get(srcAxis, 'Children'), 'XData');
        yData = get(get(srcAxis, 'Children'), 'YData');
        hLine = findobj(srcAxis, 'Type', 'Line'); %For colors and linestyle

        if ~iscell(xData)
        % If it's not a cell array, convert it to a cell array
            xData = {xData};
            lineStyle = {(get(hLine, 'LineStyle'))};
            colorStyle = {(get(hLine, 'Color'))};
        else
        % If it's already a cell array, leave it as is
            xData = xData;
            lineStyle = flip(get(hLine, 'LineStyle'));
            colorStyle = flip(get(hLine, 'Color'));
        end
        if ~iscell(yData)
        % If it's not a cell array, convert it to a cell array
            yData = {yData};
        else
        % If it's already a cell array, leave it as is
            yData = yData;
        end
        xData{i} = xData;
        yData{i} = yData;
        set(gcf,'WindowStyle','normal')
        set(gcf, 'Position', get(0, 'Screensize'));
        % Get labels, title, and legends from the individual plot
        xLabel = get(get(srcAxis, 'XLabel'), 'String');
        yLabel = get(get(srcAxis, 'YLabel'), 'String');
        plotTitle = get(srcAxis, 'Title').String;
        legendText = get(get(srcAxis, 'Legend'), 'String');

        % Calculate the subplot position based on the current iteration
        subplotPosition = i;
        if i > subplotRows * subplotCols
            subplotPosition = i - (subplotRows * subplotCols);
        end
        
        % Create subplot in the new figure and plot the data
        subplotHandle = subplot(subplotRows, subplotCols, subplotPosition);
        defaultColorOrder = get(gca, 'ColorOrder');
        yData{i} = flip(yData{i});
        
        for n = 1:length(xData{i})
            hold on
            plot(xData{i}{n}, yData{i}{n}, 'LineWidth', 1.5,'Color',colorStyle{n},'LineStyle',lineStyle{n});
        end

        xlabel(xLabel);
        ylabel(yLabel);
        xlim(get(srcAxis, 'Xlim'))
        ylim(get(srcAxis, 'Ylim'))
        xticks(get(srcAxis, 'XTick'));
        yticks(get(srcAxis, 'YTick'));
        xticklabels(get(srcAxis, 'XTickLabel'));
        yticklabels(get(srcAxis, 'YTickLabel'));

        title(plotTitle);
        if ~isempty(legendText)
            legend(legendText{n});
        end

        % Change font size for the subplot title
        set(subplotHandle, 'FontSize', 9)
        % Customize font size for x-ticks and y-ticks
        set(subplotHandle.XAxis, 'FontSize', 10); % Set font size for X-ticks
        set(subplotHandle.YAxis, 'FontSize', 10); % Set font size for Y-ticks
        % Labels
        set(subplotHandle.XLabel, 'FontSize', 10); % Set font size for x-axis labels
        set(subplotHandle.YLabel, 'FontSize', 10); % Set font size for y-axis labels
        set(subplotHandle.Legend, 'FontSize', 10); % Set font size for legend
        if ~isempty(legendText)
            legend(subplotHandle, legendText{:}, 'Location', 'Best'); % You can change 'Best' to any other valid position specifier  
        end
        grid on
    end
end
