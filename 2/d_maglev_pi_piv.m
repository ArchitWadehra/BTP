%
%% D_MAGLEV_PI_PIV
%
% Designs a PI current control and a PIV ball position control for the
% MAGLEV system.
%
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.
% 
function [ Kp_c, Ki_c, Kp_b, Kv_b, Ki_b, Kff_b, Gc, Gxb1, ic0 ] = d_maglev_pi_piv( Lc, Rc, Km, Rs, Mb, g, xb0, PO_c, tp_c, PO_b, ts_b, p0  )
% 
%% I) Inner PI Coil Current Loop
% Initialization of the Laplace Representation of the coil system
% open-loop TF: Gc = Ic/Vc 
% DC gain [A/V]
% Kc_dc = 1 / ( Rc + Rs );
Kc_dc = 1 / Rc;
% time constant [s]
% tau_c = Lc / ( Rc + Rs ); % = 0.0375
tau_c = Lc / Rc; 
% open-loop TF: Gc = Ic/Vc
Gc_num = [ Kc_dc ];
Gc_den = [ tau_c, 1 ];
% OL system
Gc = tf( Gc_num, Gc_den );
% 
% CURRENT LOOP
% Calculate the required controller gains, Kp_c and Ki_c, that meeting the 
% desired specifications
% i) Maximum Percent Overshoot (PO_c)
if ( PO_c > 0 )
    % PO_c = 100 * exp( - pi * zeta_c / sqrt(  1 - zeta_c^2 ) )
    zeta_c_min = abs( log( PO_c / 100 ) ) / sqrt( pi^2 + log( PO_c / 100)^2 );
    zeta_c = zeta_c_min;
else
    error( 'Error: Set Percentage Overshoot.' )
end
% ii) Peak Time (tp_c)
wn_c = pi / ( tp_c * sqrt( 1 - zeta_c^2 ) );
% 
% Compute the PI controller gains 
Kp_c = (2*zeta_c*wn_c*tau_c - 1) / Kc_dc;
Ki_c = wn_c^2 * tau_c / Kc_dc;
% 
% PI controller TF
G_pi_c = tf( [ Kp_c, Ki_c ], [ 1, 0 ] );
% overall closed-loop TF: Tc = Ic / Ic_des
Gc_pi_c = series( Gc, G_pi_c );
Tc = feedback( Gc_pi_c, 1, -1 );
% 
% at the selected quiescent point of operation, corresponding to (xb0, Ic0):
% Operating Coil Current (A): at ball equilibrium for (xb0, Ic0): Fc = Fg
ic0 = sqrt( 2 * Mb * g / Km ) * xb0;
% 
%% II) Outer Ball Position Control
% In a first approach to calculate 'Gxb = xb / Ic'
% we assume: 'Ic = Ic_des' at all times, i.e. 'Tc = Ic / Ic_des = 1'
% Initialization of the Laplace Representation of the mechanical plant (i.e. steel ball)
% open-loop TF: Gxb = xb1 / Ic1 
% DC gain [m/A]
Kb_dc = xb0 / ic0; % = 0.0064
% open-loop natural frequency [rad/s]
wb = sqrt( 2 * g / xb0 ); % = 57.2 rad/s
% Gxb1 = xb1/Ic1 Laplace TF
Gxb1_num = [ - Kb_dc * wb^2 ];
Gxb1_den = [ 1, 0, - wb^2 ];
% OL system
Gxb1 = tf( Gxb1_num, Gxb1_den );
%
% feed-forward gain [A/m]
Kff_b = ic0 / xb0;
% 
% POLE PLACEMENT: POSITION LOOP
% calculate pb1 and pb2 from PO_b, ts_b
% i) spec #1: maximum Percent Overshoot (PO_b)
if ( PO_b > 0 )
    zeta_b_min = abs( log( PO_b / 100 ) ) / sqrt( pi^2 + log( PO_b / 100)^2 );
    zeta_b = zeta_b_min;
else
    error( 'Error: Set Percentage Overshoot.' )
end
% 
% ii) spec #2: ts_b:
% 2% settling time: ts_b = 4 / ( zeta_b * wn_b )
wn_b = 4 / ( zeta_b * ts_b );
% 
% Ball Position PIV controller gains
Kp_b = -(2*p0*zeta_b*wn_b*xb0 + wn_b^2*xb0+2*g)*ic0/(2*xb0*g);
Kv_b = -(p0+2*zeta_b*wn_b)*ic0/(2*g);
Ki_b = -p0*wn_b^2*ic0/(2*g);

% PI position controller (pc) TF
Gpc_b = parallel( Kp_b, tf( Ki_b, [ 1, 0 ] ) );
% inner velocity feedback
Gv_b = tf( [ Kv_b, 0 ], [ 1 ] );
Gxb_v = feedback( Gxb1, Gv_b, -1 );
% CL P(I)V TF
Gxb_v_pc = series( Gpc_b, Gxb_v );
Gxb_cont = feedback( Gxb_v_pc, 1, -1 );
% Feed-Forward TF: Ic_ff = Kff_b * xb_des
Gff = tf( [ Kff_b ], [ 1 ] );
Gff_eq = parallel( Gff / Gpc_b, 1 );
% CLTF: P(I)V + FF
Txb_mat = series( Gff_eq, Gxb_cont );

end


