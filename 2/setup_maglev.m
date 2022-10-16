%
%% SETUP_LAB_MAGLEV_PIV
%
% Magnetic Levitation (MAGLEV) Control Lab: 
% Design of a PI Coil Current Control Loop 
% Nested Within a PIV-plus-FeedForward Ball Position Control Loop.
% 
% SETUP_LAB_MAGLEV_PIV sets the MAGLEV system's 
% model parameters accordingly to the user-defined configuration.
% SETUP_LAB_MAGLEV_PIV can also set the both controllers' parametersd, 
% accordingly to the user-defined desired specifications.
%
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%% MAGLEV CONFIGURATION
% Amplifier Gain used: set VoltPAQ gain switch to 3
K_AMP = 3;
% Amplifier (AMP) Type: set to 'VoltPAQ'
AMP_TYPE = 'VoltPAQ';
%
%% CONTROLLER DESIGN
% Coil Current Control Loop: percent pvershoot and peak time 
% specifications 
PO_c = 1.5;
tp_c = 0.015;
% Ball Position Control Loop: percent overshoot, settling time, and
% pole location specifications
PO_b = 5;
ts_b = 0.3;
p0 = 30;
%
% at the selected quiescent point of operation:
% Operating Air Gap (m)
xb0 = 6e-3;
% Rising (and Falling) Setpoint Slew Rate (m/s)
XB_REF_RATE = 5e-3;
%
% Specifications of the second-order low-pass filter
% for the ball (differentiated) velocity signal
wcf = 2 * pi * 75;     % filter cutting frequency (rad/s)
zetaf = 0.9;        % filter damping ratio
% Specifications of the first-order low-pass filter
% for the ball position feedback signal
tau_xb = 1 / ( 2 * pi * 80 );     % filter time constant (s)
% first-order low-pass filter specifications 
% used before plotting the command voltage (Vc) and the coil current (Ic)
tau_c = 1 / ( 2 * pi * 10 );     % filter time constant (s)
% Integral anti-windup maximum for the limiter integrator
% in the ball position loop (A)
MAX_XB_WDUP = 1;

%% MAGLEV PARAMETERS
% Set the model parameters accordingly to the user-defined MAGLEV system configuration.
% These parameters are used for model representation and controller design.
[ Lc, Rc, Km, Rs, Mb, Tb, g, K_B, IC_MAX, VMAX_AMP, IMAX_AMP ] = config_maglev( AMP_TYPE );
% 
%% DISPLAY
% Automatically calculate the controller gains for both current and position control loops
% from the desired closed-loop pole locations
% Identified resistance (ohm)
Rc = 10.0;
% Identified inductance (H)
Lc = 0.374;
% 
[ Kp_c, Ki_c, Kp_b, Kv_b, Ki_b, Kff_b, Gc, Gxb1, ic0 ] = d_maglev_pi_piv( Lc, Rc, Km, Rs, Mb, g, xb0, PO_c, tp_c, PO_b, ts_b, p0 );    
% Display the calculated gains
disp( ' ' )
disp( 'Current loop - Calculated PI controller gains: ' )
disp( [ '   Kp_c = ' num2str( Kp_c ) ' V/A' ] )
disp( [ '   Ki_c = ' num2str( Ki_c ) ' V/s/A' ] )
disp( 'Position loop - Calculated PIV controller gains: ' )
disp( [ '   Kff_b = ' num2str( Kff_b ) ' A/m' ] )
disp( [ '   Kp_b = ' num2str( Kp_b ) ' A/m' ] )
disp( [ '   Kv_b = ' num2str( Kv_b ) ' A-s/m' ] )
disp( [ '   Ki_b = ' num2str( Ki_b ) ' A/s/m' ] )