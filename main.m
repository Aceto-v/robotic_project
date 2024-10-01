%%% main %%%

%Progetto Robotica Industriale - Prendibot
%Progetto a cura degli alunni Vincenzo Maria Fiorentino - Armando Quatra
% Progettare e simulare il movimento di un braccio robotico 6DOF
% Compito assegnato: parti dalla posizione iniziale P0 -> vai alla
% posizione P1 e prendi oggetto -> sposta oggetto alla posizione P2 ->
% sposta oggetto alla posizione P3 -> torna alla posizione iniziale P0

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