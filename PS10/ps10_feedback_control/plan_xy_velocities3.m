%invent a proposed schedule of hand velocities; specify them at fixed intervals DT
%MUST start at 0 and end at 0
%try a trapezoid

function [v_xy_plan] = plan_xy_velocities3 (amax,vmax,x_trans,DT)
%choose npts:
%vmax = 1.0;
%amax= 1.0;
%x_trans =2; %translate this far

t_ramp = vmax/amax
d_ramp = 0.5*amax*t_ramp*t_ramp
d_cruise = x_trans-2*d_ramp
t_cruise = d_cruise/vmax
if (d_cruise<0) %should be triangular; so adjust vmax
  d_cruise=0;
d_ramp = x_trans/2;
t_ramp = sqrt(x_trans/amax);% x_trans/2 = 0.5*amax*tramp^2
vmax = amax*t_ramp; %will only get to this peak velocity
end

%fill in trapezoid w/ time steps DT
%tf = t_ramp+t_ramp+t_cruise;
nsteps_ramp = round(t_ramp/DT) %use a discrete number of steps for ramp
t_ramp = DT*nsteps_ramp
amax = vmax/t_ramp % adjust accel, if necessary, to make ramp a discrete number of steps
d_ramp=0.5*amax*(DT*nsteps_ramp)*(DT*nsteps_ramp)
d_cruise =x_trans - 2*d_ramp
t_cruise = d_cruise/vmax
nsteps_cruise = round(t_cruise/DT)
nsteps_tot = 2*nsteps_ramp+nsteps_cruise
v_xy_plan = zeros(2,nsteps_tot);
%ramp up:
for i=1:nsteps_ramp
  v_xy_plan(1,i) = (i-1)*DT*amax;
end
%braking:
for i=nsteps_tot:-1:nsteps_tot-nsteps_ramp
  v_xy_plan(1,i)= (nsteps_tot-i)*DT*amax;
end
%cruise
for i=nsteps_ramp+1:nsteps_tot-nsteps_ramp-1
  v_xy_plan(1,i)=vmax;
end

