%% =========================================================================
%  VTOL Helicopter Rig — LQR-I Controller Design
%  =========================================================================
%
%  This script constructs the linearized state-space model of the 3-DOF
%  helicopter rig, augments it with integral states for elevation and yaw,
%  verifies controllability, and solves the algebraic Riccati equation to
%  obtain the optimal LQR-I gain matrix K.
%
%  The resulting K matrix is exported to CSV for direct use in LabVIEW.
%
%  State vector (original 6 states):
%    x = [e, theta, psi, e_dot, theta_dot, psi_dot]
%    e       : elevation angle (rad)
%    theta   : roll angle      (rad)
%    psi     : yaw angle       (rad)
%    e_dot   : elevation rate  (rad/s)
%    theta_dot : roll rate     (rad/s)
%    psi_dot : yaw rate        (rad/s)
%
%  Input vector:
%    tau = [u, v]
%    u = F1 + F2  : collective thrust
%    v = F1 - F2  : differential thrust (roll torque)
%
%  Augmented state vector (8 states):
%    x_aug = [e, theta, psi, e_dot, theta_dot, psi_dot, int_e, int_psi]
%
%  Authors: Grati Elyes, Njeh Oussema, Snoun Ferid, Khelil Souheib
%  Class:   IIA4
% =========================================================================

%% Physical Parameters
% -------------------------------------------------------------------------
l     = 0.56;       % Arm length (m)
d     = 0.14;       % Motor lateral offset from central axis (m)
m     = 0.03;       % Effective mass at arm tip (kg)
g     = 9.81;       % Gravitational acceleration (m/s^2)
u0    = m * g;      % Hover equilibrium thrust (N) — linearization point

J_psi   = 0.23;     % Moment of inertia about yaw axis   (kg·m^2)
J_e     = 0.21;     % Moment of inertia about elevation axis (kg·m^2)
J_theta = 0.0032;   % Moment of inertia about roll axis   (kg·m^2)
epsilon = 0.2;      % Input coupling coefficient (strong coupling, eps != 0)

%% Linearized System Matrix A (6x6)
% -------------------------------------------------------------------------
%  Linearized around equilibrium: e=0, theta=0, psi=0, all rates=0
%  The nonzero entry A(6,2) = l*u0/J_psi comes from the psi_ddot equation:
%    J_psi * psi_ddot = l*(u*sin(theta) + eps*v*cos(theta))*cos(e)
%  Linearizing: sin(theta)~theta, cos(theta)~1, cos(e)~1 gives:
%    psi_ddot = (l*u0/J_psi)*theta
%  which creates the coupling A(6,2).

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, (l*u0)/J_psi, 0, 0, 0, 0];

%% Input Matrix B (6x2)
% -------------------------------------------------------------------------
%  B columns correspond to [u, v] inputs respectively.
%  Row 4 (e_ddot):   l/J_e from collective thrust u
%  Row 5 (theta_ddot): d/J_theta from differential thrust v
%  Row 6 (psi_ddot):   l*eps/J_psi from differential thrust v (coupling)

B = [0,        0;
     0,        0;
     0,        0;
     l/J_e,    0;
     0,        d/J_theta;
     0,        (l*epsilon)/J_psi];

%% Integral Augmentation
% -------------------------------------------------------------------------
%  We integrate elevation (state 1) and yaw (state 3) to eliminate
%  steady-state errors caused by model mismatch and constant disturbances.
%
%  C_int selects which states are integrated:
%    Row 1: picks e     (state 1)
%    Row 2: picks psi   (state 3)

C_int = [1, 0, 0, 0, 0, 0;   % integrates e
         0, 0, 1, 0, 0, 0];   % integrates psi  — size: 2x6

%% Augmented System Matrices (8x8 and 8x2)
% -------------------------------------------------------------------------
%  The augmented state satisfies:
%    d/dt [x; xi] = A_aug * [x; xi] + B_aug * tau
%  where xi = [int_e; int_psi] and xi_dot = -C_int * x (error integration)

A_aug = [A,      zeros(6,2);
         -C_int, zeros(2,2)];  % 8x8

B_aug = [B;
         zeros(2,2)];           % 8x2

%% Controllability Check
% -------------------------------------------------------------------------
Co_aug     = ctrb(A_aug, B_aug);
rank_Co_aug = rank(Co_aug);

fprintf('Controllability rank of augmented system: %d / 8\n', rank_Co_aug);
if rank_Co_aug == 8
    fprintf('System is fully controllable. Proceeding with LQR design.\n\n');
else
    error('System is NOT fully controllable. Check model parameters.');
end

%% LQR Weight Matrices
% -------------------------------------------------------------------------
%  Q weights (8 states): [e, theta, psi, e_dot, theta_dot, psi_dot, int_e, int_psi]
%
%  Tuning rationale:
%    theta (300): Roll instability couples directly into yaw via A(6,2).
%                 High weight ensures fast roll stabilization.
%    psi   (150): Primary position output — tight regulation desired.
%    theta_dot (100): Aggressive rate damping prevents oscillations.
%    e_dot  (1): Elevation rate is naturally damped; low weight sufficient.
%    int_e  (30): Moderate integral action — elevation has slow drift.
%    int_psi(20): Integral action on yaw — eliminates motor asymmetry bias.
%
%  R = diag([300, 300]):
%    Equal penalty on collective and differential thrust.
%    High value (300) limits motor saturation and prevents aggressive inputs.

Q = diag([100, 300, 150, 1, 100, 5, 30, 20]);
R = diag([300, 300]);

%% Solve Algebraic Riccati Equation
% -------------------------------------------------------------------------
[K, S, eigenvalues] = lqr(A_aug, B_aug, Q, R);

fprintf('Optimal gain matrix K (2x8):\n');
disp(K);

fprintf('Closed-loop eigenvalues:\n');
disp(eigenvalues);

%% Verify Closed-Loop Stability
% -------------------------------------------------------------------------
A_cl = A_aug - B_aug * K;
cl_eigs = eig(A_cl);
if all(real(cl_eigs) < 0)
    fprintf('All closed-loop eigenvalues have negative real parts. System is stable.\n');
else
    warning('Some closed-loop eigenvalues are non-negative. Review Q/R tuning.');
end

%% Export Gain Matrix for LabVIEW
% -------------------------------------------------------------------------
%  K is exported as a CSV file for direct import into the LabVIEW VI.
%  Format: 2 rows (one per input), 8 columns (one per augmented state).

writematrix(K, 'export_data.csv');
fprintf('\nGain matrix exported to export_data.csv\n');
fprintf('Load in LabVIEW: Read From Spreadsheet File → 2x8 numeric array\n');
