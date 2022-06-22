%% Define Vehicle Parameters
cs = 180;
cf = cs;  % N/rad
cr = cs;  % N/rad
I = 0.093;  % Izz, kg*m^2

b = 0.147; % m
a = 0.164;
m = 45.92/9.81;
% u = 1
u = 1.713; % m/s


%% Get Time and Input series
test = tdfread('RandomDrive.txt','\t');
t = test.t;
U = -(0.311/1.08)*(test.steer-0.0);
dt = 0.005; % approximately
InputSeries = [t U];
y2n = [test.Gr*(2*pi/360) -1*test.Ay];
U = U';

%% Initiate Kalman Matrices
CovarShape = [1 0; 0 1];

% Create initial state and covariance matrices
X_0 = [0;0];
P_0 = 0*CovarShape;

%Process error matrix
Q = 0.5*CovarShape;

%Observation error matrix
R = 0.5*CovarShape;

%% Define System Model
% Kalman bicycle model
A = [-(cf+cr)/((m)*(u)) -(u)-(a*cf-b*cr)/((m)*(u));
        -(a*cf-b*cr)/((I)*(u)) -((a^2)*cf+(b^2)*cr)/((I)*(u)) ];

B = [cf/(m); a*cf/(I)];

% Outputs are states 
C = [1 0;
     0 1];

% U does not change state
D = [0;
    0];

ksys = ss(A,B,C,D); % Outputs states [v;r]
sysD = c2d(ksys,dt); 
Adt = sysD.A;
Bdt = sysD.B;
Cdt = sysD.C;
Ddt = sysD.D;

%% Begin Kalman Loop

% Set initial values
X_k0 = X_0;
P_k0 = P_0;

% Hold Predictions Matrix
K_preds = zeros(size(y2n));
K_Xs = zeros(size(y2n));
K_dots = zeros(size(y2n));
GT_dots = zeros(size(y2n));
K11_gains = zeros(size(y2n));
K12_gains = zeros(size(y2n));
K21_gains = zeros(size(y2n));
K22_gains = zeros(size(y2n));
P_kps = zeros(size(y2n,1),4);
P_k0s = zeros(size(y2n,1),4);

for i = 1:size(U,2) % For every input
    
    
    % State Extrapolation
    % X_kp1 = A*X_k0 + B*U_k1 + w_k
    X_kp1 = Adt*X_k0 + Bdt*U(i);
    
    % P_kp = A*P_k0*A' + Qk
    P_kp = Adt*P_k0*transpose(Adt) + Q;
    P_kps(i,:) = [P_kp(1,:) P_kp(2,:)];
    
    % Determine Kalman Gain
    % K = (P_kp*C')/(C*P_kp*C' + R)
    K = (P_kp*transpose(Cdt))*inv(Cdt*P_kp*transpose(Cdt) + R);
    K11_gains(i,:) = K(1,1);
    K12_gains(i,:) = K(1,2);
    K21_gains(i,:) = K(2,1);
    K22_gains(i,:) = K(2,2);

    % State Correction
    % X_k1 = X_Kp1 + K[Y_k - C*X_kp]
    % must convert IMU measurement of r and ay to state [v;r] to determine
    % error/innovation between system extrapolation and measurement
    % implication
    % ay = v_dot + ru
    % v_dot = ay - ru
    % v_dot = A11*v + A12*r + B11*u
    % v = (v_dot - A12*r - B11*u)/(A11)
    % therefore: v_measured = (ay - ru - A12*r - B11*U)/A11
    v_measured = (y2n(i,2) - y2n(i,1)*u - A(1,2)*y2n(i,1) - B(1,1)*U(i))/A(1,1);

    X_k1 = X_kp1 + K*([v_measured;y2n(i,1)] - Cdt*X_kp1); % [v;r]measured - [v;r]extrapolated
    K_Xs(i,:) = X_k1;

    %         r             A11*v        A12*r                B11*U     r*u
    Y_k1 = [X_k1(2); A(1,1)*X_k1(1) + A(1,2)*X_k1(2) + B(1)*U(i) + X_k1(2)*u]; % from corrected state determine kalman predicted measurement [r;ay]
    K_preds(i,:) = transpose(Y_k1); % [r;ay]
    k_pDot = Adt*X_k1 + Bdt*U(i);% Kalman prediction of state change
    K_dots(i,:) = transpose(k_pDot);
    

    % Current Becomes Previous
    X_k0 = X_k1;
    P_k0 = (eye(2)-K*Cdt)*P_kp;
    P_k0s(i,:) = [P_k0(1,:) P_k0(2,:)]; 
end

% Plot noisy measurements and kalman predictions
figure('Name','Kalman Predicted Measurements vs time')
% plot(t,y2,'-');
% hold on
plot(t,y2n,'x');
hold on
plot(t,K_preds,'-','LineWidth',2)
plot(t,test.steer,'-','LineWidth',2)
% legend('Measured_r','Measured_a_y')%,'Kalman Predicted_1_,_r','Kalman Predicted_2_,_a_y');
legend('M_r','M_a_y','K_1_,_r','K_2_,_a_y','steer');



figure('Name','Kalman Predicted States vs time')
beta = K_Xs(:,1)/u;
plot(t,K_Xs,'--');
hold on
plot(t,beta,'-');
plot(t,test.steer,'-');
legend('K_v','K_r','K_b','Steer');

% figure('Name','Kalman Predicted Dots vs time')
% plot(t,K_dots,'--')
% legend('K_v_-_d_o_t','K_r_-_d_o_t')
% 
% 
% figure('Name','Kalman Gain vs time')
% plot(t,K11_gains)
% hold on
% plot(t,K12_gains)
% plot(t,K21_gains)
% plot(t,K22_gains)
% legend('K11','K12','K21','K22')
% 
% figure('Name','P_kp vs time')
% plot(t,P_kps)
% 
% figure('Name','P_k0 vs time')
% plot(t,P_k0s)


%% attempt time history solution
dt = zeros(size(t));
heading = zeros(size(t));
x = zeros(size(t));
y = zeros(size(t));
for i = 1:size(t)-1
    dt(i) = t(i+1) - t(i);
    heading(i+1) = heading(i) + K_Xs(i,2)*dt(i);
    x(i+1) = x(i) + (u*cos(heading(i)) + K_Xs(i,1)*sin(heading(i)))*dt(i);
    y(i+1) = y(i) + (u*sin(heading(i)) + K_Xs(i,1)*cos(heading(i)))*dt(i);
end
figure('Name','Time History Solution')
plot(x,y)