%% DRAW KUKA
clear all;
clc;
close all;

%load("PROVA_WORKSPACE.mat");
%load("new_workspace.mat");
load("workspace_08-Nov-2023.mat");
kuka = importrobot('./models/kuka_lwr.urdf');

addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")

kuka.Gravity = [0,0,-9.81];
kuka.DataFormat = 'row';

joints = load("Q.txt");

figure;
rateCtrlObj = rateControl(10000);

H = [0 0.1 0 0.2 -1 0 0];

% prova = kuka.show(H,'visuals','on','collision','off');
% prova.CameraPosition = [-2,7,6];

for i=1:length(joints)    
    %time(i)
    prova = kuka.show(joints(i,1:7),'visuals','on','collision','off');
    hold on;
    %% TEST 2 (CIRCLE)
    prova.CameraPosition = [-2,7,6];    
    %scatter3(task_vec(:,10),task_vec(:,11),task_vec(:,12),[8],'red');
    %% PUT IN THE PLOT ALSO THE REFERENCE TRAJECTORY
    waitfor(rateCtrlObj);
    hold off;
end