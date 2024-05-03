% MAE 547 Final Project
% run this script to access the GUI
clear all
disp("Welcome to MAE547 Final Project. Refer to the GUI screen for next steps")
warning('off')
addpath(".\helper_functions");
addpath(".\rvctools");
run("startup_rvc.m")
hGui = gui_guide;
handles = guidata(hGui);
guidata(hGui, handles);

% assumed variables
compliance_gains =[100 25 10 ;75 75 75; 50 10 1];
impedance_gains =[100 25 10 ;75 75 75; 50 10 1; 10 10 10; 20 20 20];
gravity_mat =[0 0 -9.81];
des_time =100;