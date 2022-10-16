%
%% CONFIG_MAGLEV
%
% Returns the amplifier, sensor, and model parameters of the MAGLEV plant.
%
% MAGLEV system nomenclature:
% Lc        Coil Inductance                     (H)
% Rc        Coil Resistance                     (Ohm)
% Km        Electromagnet Force Constant        (N.m^2/A^2)
% Rs        Current Sense Resistance            (Ohm)
% Mb        Ball Mass                           (kg)
% g         Gravity Constant                    (m/s^2)
% K_B       Ball Position Sensor Sensitivity    (m/V)
% VMAX_AMP  AMP Maximum Output Voltage          (V)
% IMAX_AMP  AMP Maximum Output Current          (A)
% rb        Steel Ball Radius                   (m)
% Tb        Steel Ball Travel                   (m)
% Nc        Number Of Turns in the Coil Wire
% rc        Coil Core Radius                    (m)
% mu0       Magnetic Permeability Constant      (H/m)
%
% Copyright (C) 2003 Quanser Consulting Inc.
% Quanser Consulting Inc.


%%
function [ Lc, Rc, Km, Rs, Mb, Tb, g, K_B, IC_MAX, VMAX_AMP, IMAX_AMP ] = config_maglev( AMP_TYPE )
% from Inch to Meter
K_IN2M = 0.0254;
% Coil Inductance (H)
Lc = 0.4125;
% Coil Resistance (Ohm)
Rc = 10;
% Number Of Turns in the Coil Wire
Nc = 2450;
% Coil Length (m)
lc = 3.25 * K_IN2M; % = 0.0825
% Coil Core Radius (m)
rc = 8e-3;
% Electromagnet Force Constant (N.m^2/A^2)
Km = 6.5308e-5;
% Current Sense Resistance (Ohm)
Rs = 1;
% Ball Mass (kg)
Mb = 0.068;
% Steel Ball Radius (m)
rb = K_IN2M / 2; % = 0.0127
% Steel Ball Travel (m)
Tb = 0.014;
% Gravitational Constant on Earth (m/s^2)
g = 9.81;
% Magnetic Permeability Constant (H/m)
mu0 = 4 * pi * 1e-7;
% Ball Position Sensor Sensitivity (m/V)
% the ball position sensor  voltage is calibrated to go from 0V to +4.95V in 14mm (Tb)
K_B = Tb / 4.75;
% Set the AMP Maximum Output Voltage (V) and Output Current (A)
% rm: for low values of K_CABLE, VMAX_AMP is limited by VMAX_DAC
if strcmp( AMP_TYPE, 'VoltPAQ' )
    VMAX_AMP = 24;
    IMAX_AMP = 5;
else
    error( 'Error: Set the amplifier type for the Magnetic Levitation experiment.' )
end
% maximum coil current (A)
IC_MAX = 3;
% end of 'setup_maglev_configuration( )'