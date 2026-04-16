l = 0.56; 
d = 0.14;
m = 0.03;
u0 = m*9.81;
J_psi = 0.23;
J_e = 0.21;
J_theta = 0.0032;
epsilon = 0.2; 

%% A Matrix (6x6)
A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0,(l*u0)/J_psi, 0, 0, 0, 0];

%% B Matrix (6x2)
B = [0, 0;
     0, 0;
     0, 0;
     l/J_e, 0;
     0, d/J_theta;
     0, (l*epsilon)/J_psi];

%% Integrate state 1 and state 3
C_int = [1, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0];   % 2x6

%% Augmented system (8 states total)
A_aug = [A, zeros(6,2);
         -C_int, zeros(2,2)];

B_aug = [B;
         zeros(2,2)];

%% Controllability check
Co_aug = ctrb(A_aug, B_aug);
rank_Co_aug = rank(Co_aug);

disp(['Rank of augmented controllability matrix: ', num2str(rank_Co_aug)]);

%% LQR weights (8 states now)
Q = diag([100, 300, 150 ,1 , 100, 5, 30, 20]); 
R = diag([300, 300]); 

[K, S, e] = lqr(A_aug, B_aug, Q, R);

disp(K);
writematrix(K, 'export_data.csv');