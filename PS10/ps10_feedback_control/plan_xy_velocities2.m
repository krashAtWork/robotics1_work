%invent a proposed schedule of hand velocities; specify them at fixed intervals DT
%MUST start at 0 and end at 0

function [v_xy_plan] = plan_xy_velocities2 (tf,DT)
%choose npts:
%npts=2000;
a = pi/tf;
npts = tf/DT; 
w = 2*a;
%vmax=0.2; 
%dv = vmax/(npts/2);
%create a plan for hand velocity as a function of time with timestep intervals DT
v_xy_plan = zeros(2,npts);
for i=2:npts
  v_xy_plan(1,i) = abs(a*sin(w*(i-1)*DT));
end

v_xy_plan(1,1)=0.0; %required start/end velocity
v_xy_plan(1,npts)=0.0;
end
