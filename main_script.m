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
