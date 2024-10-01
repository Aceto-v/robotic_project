%%% main %%%
clear all
close all
clc

%% 0. Setup
addpath('data');
addpath("functions");
addpath("video");

%% 1. Execute Workspace Analysis

Prendibot_workspace;

%% 2. Execute Inverse Kinematics

Prendibot_invKin_sym;

%% 3. Simulation

Prendibot_simulation;

%% 4. Plot

Prendibot_graphics_analysis;