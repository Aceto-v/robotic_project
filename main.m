%%% main %%%
clear all
close all
clc

%% 0. Setup
addpath('data');
addpath("functions")

%% 1. Execute Workspace Analysis
Prendibot_FV_workspace;

%% 2. Execute Inverse Kinematics
Prendibot_FV_invKin_sym;

%% 3. Simulation
Prendibot_FV_num;

%% 4. Plot
Prendibot_FV_graphics;