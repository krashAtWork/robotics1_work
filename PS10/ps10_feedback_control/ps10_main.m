%simulated dynamics with control:

%initialize the system: q(0), qdot(0)
%create a plan for desired q_des(t), qdot_des(t)

%"stream" q_des(t)  (and qdot_des(t)
%compare errs: q_err = q_des(t) - q(t)
%set Kp = [Kp1,0;0,Kp2];
%set Kv = [Kv1,0;0,Kv2];
%command tau = Kp*q_err(t) -Kv*qdot(t)
%march forward: qddot = H_inv*(tau - c_vec)
%qdot+= qddot*dt
%q+=qdot*dt

%simulation of 2DOF arm dynamics
%control path along x=-1 to x=+1 at y=0.2
clear all
tau_vec_max = [10;5]

a_vec = [1;1]; %link lengths
grav_vec=[0;0;9.8]; %gravity is parallel to joint axes, so no gravity torques

%CONTROL GAINS:
Kp = [1,0;0,1];  %TUNE ME!!
Kv = [1,0;0,1]; %TUNE ME!!

%specify the desired hand path: a line parallel to the x-axis, offset by y=0.2
%do not change the following path parameters
x_start=-1;
x_end = 1;
y=0.1;
p_start=[x_start;y];
ps_vec = [x_end-x_start;0] %this is the tangent of the task-space path
x_trans=x_end-x_start %move hand this far in the x direction
hand_xy_start = [x_start;y];
DT=0.01  %the controller is updated at 100Hz, i.e. every 10msec

%YOU MAY CHANGE THE NEXT 2 PARAMETERS TO MODIFY THE PATH PLAN
amax=0.5 
vmax=0.4

v_xy_plan = plan_xy_velocities3 (amax,vmax,x_trans,DT);
[dummy,npts]=size(v_xy_plan)
time_vec=0:1:npts-1;
time_vec=time_vec*DT;
figure(1)
clf
plot(time_vec,v_xy_plan(1,:),'r',time_vec,v_xy_plan(2,:),'b');
title('planned vx(t) (red) and vy(t) (blue)')
xlabel('time (sec)')
ylabel('vel (m/s)')


accel_xy_plan = accel_from_vel_plan(v_xy_plan,DT);
hand_xy_plan = hand_xy_from_v_xy(hand_xy_start,v_xy_plan,DT);
time_vec=0:DT:(npts-1)*DT;
figure(2)
plot(time_vec,hand_xy_plan(1,:),'r',time_vec,hand_xy_plan(2,:),'b');
title('planned x(t) (red) and y(t) (blue)')
xlabel('time (sec)')
ylabel('distance (m)')




figure(3)
plot(time_vec,accel_xy_plan(1,:),'r',time_vec,accel_xy_plan(2,:),'b');
title('planned accelerations: ax(t) (red) and ay(t) (blue)')
xlabel('time (sec)')
ylabel('accel (m/s/s)')


q_des_vecs=zeros(2,npts);
for isamp=1:npts
  hand_xy=hand_xy_plan(:,isamp);
  qvec = compute_IK(hand_xy,a_vec);
  q_des_vecs(:,isamp)=qvec;
end

figure(4)
plot(time_vec,q_des_vecs(1,:),'r',time_vec,q_des_vecs(2,:),'b')
title("IK planned: q1 (red), q2 (blue)")
xlabel('time (sec)')
ylabel('joint angle (rad)')

qdot_des_vecs = compute_qdot_vecs(v_xy_plan,q_des_vecs,a_vec);

figure(5)
clf
plot(time_vec,qdot_des_vecs(1,:),'r',time_vec,qdot_des_vecs(2,:),'b')
title('planned joint velocities: q1dot (red) and q2dot (blue)')
xlabel('time (sec)')
ylabel('joint velocity (rad/sec)')


q_vec=q_des_vecs(:,1);
qdot_vec=[0;0];
q_vec_hist=[q_vec(:,1)];
tau_vec_hist=[];

%compute and plot corresponding joint accels:
qddot_des_vecs = compute_qddot_vecs (accel_xy_plan,q_des_vecs, qdot_des_vecs,a_vec,DT);

%estimate the control bandwidth from initial pose:
[H,c_vec] = compute_H_and_cvec (q_vec, qdot_vec,a_vec)

wn1 = sqrt(Kp(1,1)/H(1,1))
wn2 = sqrt(Kp(2,2)/H(2,2))
samp_freq = 2*pi/DT
%2*zeta*wn = b/m
zeta1 = 0.5*Kv(1,1)/(H(1,1)*wn1)
zeta2 = 0.5*Kv(2,2)/(H(2,2)*wn2)

%compute the feedforward torques for this plan:
tau_vecs_ffwd = zeros(2,npts);
for i=1:npts
  %function [tau_vec,H,h_vals] = inv_dyn_2DOF (qddot_vec,qdot_vec,q_vec,DH_a_vec,grav_vec)
  [tau_vec,H,h_vals] = inv_dyn_2DOF(qddot_des_vecs(:,i),qdot_des_vecs(:,i),q_des_vecs(:,i),a_vec,grav_vec);
  tau_vecs_ffwd(:,i)=tau_vec;
end

%  [qddot_vecs] = compute_qddot_vecs (accel_xy_plan,q_des_vecs, qdot_des_vecs,a_vec,DT);
for i=1:npts
  t=(i-1)*DT;
  q_err = q_des_vecs(:,i)-q_vec;
  qdot_err = qdot_des_vecs(:,i)-qdot_vec;
  %tau_servo = Kp*q_err-Kv*qdot_vec;
  tau_servo = Kp*q_err+Kv*qdot_err; %with velocity fdfwd
  %computed torque:

  
  [tau_ffwd,H,c_vec,g_vec] = inv_dyn_2DOF (qddot_des_vecs(:,i),qdot_des_vecs(:,i),q_des_vecs(:,i),a_vec,grav_vec);


  %tau_vec  = tau_servo;
  %tau_vec=tau_ffwd;
  tau_vec=tau_servo+ tau_ffwd;
  tau_vec = sat_vec(tau_vec_max,tau_vec);
  
  qddot = fwd_dynamics(a_vec,q_vec,qdot_vec,tau_vec);
  qdot_vec= qdot_vec+qddot*DT;
  q_vec = q_vec+qdot_vec*DT;
  q_vec_hist=[q_vec_hist,q_vec];
  tau_vec_hist=[tau_vec_hist,tau_vec];
  
end
time_vec2=0:DT:npts*DT;
figure(6)
clf
plot(time_vec2,q_vec_hist(1,:),'r',time_vec2,q_vec_hist(2,:),'b')
hold on
plot(time_vec,q_des_vecs(1,:),'r',time_vec,q_des_vecs(2,:),'b')
title('desired and actual joint angles vs time')

actual_hand_xy=[];
for i=1:npts+1
  actual_hand =  compute_FK (q_vec_hist(:,i),a_vec);
  actual_hand_xy=[actual_hand_xy,actual_hand];
end
figure(7)
plot(time_vec2,actual_hand_xy(1,:),'r',time_vec2,actual_hand_xy(2,:),'b')
title('hand x and y vs time')
xlabel('time (sec)')

hand_err_vecs = hand_xy_plan-actual_hand_xy(:,1:npts);
figure(8)
plot(time_vec,hand_err_vecs(1,:),'r',time_vec,hand_err_vecs(2,:),'b')
title('hand errors vs time: x(r) and y(b)')
xlabel('time (sec)')

tau_vec_hist=[tau_vec_hist,[0;0]];
figure(9)
plot(time_vec2,tau_vec_hist(1,:),'r',time_vec2,tau_vec_hist(2,:),'b');
title('joint torques vs time')
xlabel('time (sec)')
ylabel('torques, (N-m)')
grid on




