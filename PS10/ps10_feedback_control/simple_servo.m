%illustration of simple servo, 1-DOF mass

clear all
%trivial robot dynamics: H11*qddot = tau
H11 = 2.0; %this much rotational inertia
tau_sat = 50 %constrain output torque to be within +/- this much torque
%control gains
Kp = 1000;
Kv = 50;
%initial conditions: start from rest
q_init=0.0;
qdot_init = 0.0;
q_goal = 2.0;

DT=0.01
%make a plan:
%here's a dumb plan:
t_final=2
[q_des_vec,qdot_des_vec,qddot_des_vec,time_vec] = step_plan (q_init,q_goal,DT,t_final);
%here's a triangular-velocity plan:
amax=10 %choose this
%[q_des_vec,qdot_des_vec,qddot_des_vec,time_vec] = triangular_vel_plan (q_init,q_goal, amax, DT);



figure(1)
subplot(3,1,1)
plot(time_vec,q_des_vec);
title('planned q(t) ')
xlabel('time (sec)')
ylabel('distance (rad)')

subplot(3,1,2)
plot(time_vec,qdot_des_vec);
title('planned qdot(t) ')
xlabel('time (sec)')
ylabel('vel (rad/s)')


subplot(3,1,3)
plot(time_vec,qddot_des_vec);
title('planned accelerations')
xlabel('time (sec)')
ylabel('accel (rad/s/s)')


wn = sqrt(Kp/H11)
fn = wn/(2*pi)
wn_inv = 1/wn
fnDt = fn*DT

%2*zeta*wn = b/m
zeta = 0.5*Kv/(H11*wn)

%simulate the motion
q=0
qdot=0
q_hist=[0];
qdot_hist=[0];
tau_hist=[0];
q_err_hist=[0];
[dummy,nsteps]=size(time_vec);
nsteps=nsteps-1;
for i=2:nsteps+1
  %t=(i-1)*DT;
  %compute tracking errors, q and qdot
  q_err = q_des_vec(i)-q; 
  qdot_err = qdot_des_vec(i)-qdot;
  
  %%%%%%%% control law goes here
  tau_servo = Kp*q_err-Kv*qdot;  %servo control law, no velocity feedforward
  %tau_servo = Kp*q_err+Kv*qdot_err; %with velocity fdfwd
  tau=tau_servo;
  tau = sat(tau_sat, tau); %saturate the torque 
  
  %compute accel from dynamics:
  qddot = tau/H11; %trivial 2nd-Order dynamics
  
  %do Euler 1-step integration for 1 time step
  qdot= qdot+qddot*DT; 
  q = q+qdot*DT;
  
  %save the results
  q_hist=[q_hist,q];
  q_err_hist=[q_err_hist,q_err];
  qdot_hist=[qdot_hist,qdot];
  tau_hist=[tau_hist,tau];
  
end

figure(2)
subplot(4,1,1)
plot(time_vec,q_des_vec,'r',time_vec,q_hist,'b');
title('planned q(t) (r) and actual q(t) (b) ')
xlabel('time (sec)')
ylabel('distance (rad)')

subplot(4,1,2)
plot(time_vec,qdot_des_vec,'r',time_vec,qdot_hist,'b');
title('planned qdot(t) (r) and actual (b) ')
xlabel('time (sec)')
ylabel('vel (rad/s)')


subplot(4,1,3)
plot(time_vec,tau_hist);
title('servo torques')
xlabel('time (sec)')
ylabel('torque (N-m)')

subplot(4,1,4)
plot(time_vec,q_err_hist);
title('q error')
xlabel('time (sec)')
ylabel('err (rad)')

return

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
title('hand errors vs time')
xlabel('time (sec)')

tau_vec_hist=[tau_vec_hist,[0;0]];
figure(9)
plot(time_vec2,tau_vec_hist(1,:),'r',time_vec2,tau_vec_hist(2,:),'b');
title('joint torques vs time')
xlabel('time (sec)')
ylabel('torques, (N-m)')
grid on




