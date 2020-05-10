%motor data%
clc;
clear all;
%thrust as a function of time%

%Ascent Motor%
Thrust = xlsread('C:\Users\jpraj\Desktop\FinalSim\AscentMotor.csv','B2:B253'); 
Time = xlsread('C:\Users\jpraj\Desktop\FinalSim\AscentMotor.csv','A2:A253');

Total_impulse = 62000;
Propellent_mass = 26.35;
Engine_int_mass = 35;


%Descent Motor%
time = xlsread('C:\Users\jpraj\Desktop\FinalSim\DescentMotor.csv','A2:A795');
thrust = xlsread('C:\Users\jpraj\Desktop\FinalSim\DescentMotor.csv','B2:B795');

total_impulse = 8848; %Ns%
propellant_mass = 3.711; %kg%
engine_ini_mass = 5; %kg%