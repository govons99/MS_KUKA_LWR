clear all;
clc;

%import lwr;
format long;

%% Adding path to useful folders

addpath('./Kinematics/'); 
% functions defined by Claudio Gaz
addpath('./Dynamics/'); 
% function for fault "generation"
addpath('./Lorenzo_functions/');
% functions 2R dynamics
addpath('./2R_Dynamics');


%% Loading KUKA URDF

%Using the URDF
kuka = importrobot('./models/kuka_lwr.urdf');

addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")

% controller = importrobot('./models/kuka_lwr_controller.urdf');
controller = importrobot('./models/kuka_lwr_controller_circle.urdf');

kuka.Gravity = [0,0,-9.81];
controller.Gravity = [0,0,-9.81];

kuka.DataFormat = 'row';
controller.DataFormat = 'row';

%% Parameters

t0=0.0;
tf=10.0;

DeltaT = 0.000001;
Ts = DeltaT;

% Array initialization
joints = [];
time = zeros(1);
diff_plot = [];
accelerations = [];
accs_ref = [];
torque_PD = [];

%% INITIAL CONFIGURATION

%q0 = [0.4,pi/2,pi/2,0.5,-1,0,0,0,0,0,0,0,0,0];
q0 = [0,1.1040,0,-0.75,-1,0,0,0,0,0,0,0,0,0];
accs = zeros(1,7);
joints = q0;

%% PD GAINS

% K = [-21.4267   -0.8640  -17.2654   -1.0133;
%      -4.8761  -17.7497   -3.9620  -12.6837];
 
% K = [-93.1726   -8.4440  -21.5335   -4.7909;
%      -17.7032  -34.4521   -6.6227  -20.5367];
 
% K = [4976 6267 113 769;
% 	-20189 -19397 -1424 -2541];

% K = [-9277.13944860092 3710.17708672289 -349.234832428913 266.395117894730 -194809.308523000 29590.8387072452 -497382.571831891 83024.4056398725 -512598.396322723 96517.0532040094 -268819.441373604 59336.0150773882 -73589.1199037631 20303.8593336261;
% -2029.21453824429 -7028.36935443040 -80.0713649245077 -523.663369755656 -42740.8155134585 -58507.6257749021 -108901.648552769 -163109.287869734 -111981.948671448 -188381.316026516 -58595.8106911847 -115090.430611396 -16021.4507995792 -39186.9823159717];

K = [-51479.1506564270 3568.82244932025 -11854.7020446130 1079.87266377154 -633338.094409722 4333.26737636959 508505.390501465 -47862.5287158016 -98358.8518500649 6135.56008409700 ;
1263.68859842101 -44137.3005591795 480.698288000380 -10896.5985110252 -9654.94397920896 -440771.759801160 -20520.6564484375 470082.073219219 1813.31207683121 -82035.0166304382 ];

global Phi Gamma

% Phi = [zeros(2,2) eye(2,2) zeros(2,6);
%        zeros(2,4) eye(2,2) zeros(2,4);
%        zeros(2,6) eye(2,2) zeros(2,2);
%        zeros(2,8) eye(2,2);
%        zeros(2,2) -9*eye(2,2) zeros(2,2) -10*eye(2,2) zeros(2,2)];
% 
% Gamma = [zeros(8,2); eye(2)];

Phi = [zeros(2,2) ones(2,2) zeros(2,2);
       zeros(2,2) zeros(2,2) ones(2,2);
       zeros(2,2) -25*ones(2,2) zeros(2,2)];

Gamma = [zeros(4,2); eye(2)];


Kp = 80;

Kd = 20;

