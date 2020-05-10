clc;
clear all;
                            %%propellent constants%%
                            
a = 0.0482;                                                                  %burn rate coefficient (in/s)
a1 = a*25.4;                                                                 %burn rate coefficient (mm/s)
n = 0.3607;                                                                  %pressure exponent
rho = 0.054352;                                                              %propellant density (lb/inch^3)
rho1 = rho*27679.9;                                                          %propellant density (kg/meter^3)
cstar = 2651;                                                                %characteristic velocity (ft/s)
cstar1 = cstar*0.3048;                                                       %characteristic velocity (m/s)
k1 = 1.117;                                                                  %specific heat at chamber
k2 = 1.1165;                                                                 %specific heat at exhaust
M1 = 55.381;                                                                 %molecular weight at the chamber (kg/kmol)
M2 = 55.805;                                                                 %molecular weight at the exhaust (kg/kmol)
R = 8314;                                                                    %gas constant (J/kmol-K)
Pa = 14.7;                                                                   %ambient pressure (psi)
Pa1 = Pa*6894.75729;                                                         %ambient pressure (pascal)
Tc = 2059;                                                                   %Flame temperature (kelvin)

                    %% dimensions of motor %%

ri1 = 5; hf1 = 0; bf1 = 0; nf = 0;                                           %grain configuration in mm
ri = ri1/25.4 
hf = hf1/25.4
bf = bf1/25.4
L1 = 150;                                                                    %propellant length (mm)
L = 150/25.4;                                                                %propellant length (inch)
Dep = 1.14173;
Dep1 = 29;                                                              %propellant external diameter (mm)
Dt1 = 7;                                                                     %throat diameter (mm)
Dt = Dt1/25.4;                                                               %throat diameter(inch)
De1 = 21;
De = De1/25.4;                                                               %exit diameter (inch)

                        %% area calculations and different cross sections%%
                        
Acs = pi*ri^2+hf*bf*nf;                                                      %cross-sectional area
Ab = ((2*pi*ri)+(2*nf*hf))*L;                                                %burn area (inch^2)
Ab1 = Ab*645.16;                                                             %burn area (m^2)
At = (pi*(Dt^2))/4                                                           %throat area (inch^2)
At1 = At*645.16;                                                             %throat area (mm^2)
Ae = (pi*(De^2))/4                                                           %exit area (inch^2)
Ae1 = Ae*645.16;                                                             %exit area (meter^2)
                                
                                     %% motor performance calculation  %%
                                     
Pc = ((rho*Ab*a*cstar)/(32.1741*At))^(1/(1-n))                               %chamber pressure (psi)
Pc1 = Pc*6894.76;                                                            %chamber pressure (pascal)
Kn = Ab/At                                                                   %burn to throat area
rb = (a*((Pc)^n))                                                            %burn rate (in/s)
rb1 = rb*25.4;                                                               %burn rate (mm/s)
M = sqrt((2/(k1-1))*(((Pc/Pa)^((k1-1)/k1))-1))                               %mach number at exit
Te = Tc/(1+(((k1-1)*M^2)/2))                                                 %temperature at exit
r = R/M1;                                                                    %gas constant/molecular weight in chamber
Ve = sqrt(((2*R*Tc)/(M2*(k2-1)))*(1-(Pa/Pc)^((k2-1)/k2)))                    %velocity at exit(m/s)
Ve1 = M*sqrt(k2*r*Te);                                                       %alternate formula
Pt = Pc*((1+((k1-1)/2))^(-k1/(k1-1)))                                        %throat pressure(psi)
Pt1 = Pt*6894.76
Tt = Tc/(1+((k1-1)/2))                                                       %throat temperature
mdot = ((Pc1*At1)/sqrt(Tc))*sqrt((k1/r)*(((k1+1)/2)^(-(k1+1)/(k1-1))))*10e-6 %mass flow rate (kg/s)
mdot1 = 2.205*mdot                                                           %lb/s
T = (mdot*Ve)                                                                %thrust
Cf = sqrt(((2*(k1^2))/(k1-1))*(((2/(k1+1))^((k1+1)/(k1-1)))*(1-((Pa/Pc)^((k1-1)/k1)))))%thrust coefficient
Isp1 = T/(mdot*9.81)                                                         %specific impulse
Isp2 = (Cf*cstar1/9.81);                                                     %alternate formula
AR = (Ae/At)                                                                 %area ratio
PR = (Pa/Pt)                                                                 %pressure ratio
mp = (1504*((pi*(Dep1^2)/4)-Acs)*L1)*10e-6                                   %mass of propellant required
PRT = Acs/At                                                                 %port to throt ratio
mfluxt = mdot/At                                                             %mass flux at the throat
