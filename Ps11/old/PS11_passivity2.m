%passivity analysis of a 1-DOF system
clear all
m1 = 1 %system model: a 1kg mass proximal to actuator
m2 = 1 %and 1kg mass distal from motor
Kt = 1000 %stiffness of transmission
Bt = 50 %damping of transmission
M=[m1,0;0,m2]
omega_n = sqrt(2*Kt/(m1+m2))

Bt_mat = [Bt,-Bt;-Bt,Bt];
Kt_mat = [Kt,-Kt;-Kt,Kt];
B_actuator = 10; %actuator has lots of friction; want to mask this with feedback



Kenv= 2000000 %23 %100 %120 %200000 %environment model: a spring of this stiffness, N/m

%desired impedance of Bdes and Kdes
%LOWER BDES IS BETTER, implies more responsive closed-loop admittance of
%the robot\
a = 4.5 ;
b = 5.8 ;
Bdes = (a + b ) /2%damping of desired admittance
Bdes = 150000;
Kdes = 0; %10; %20; %100 %stiffness of desired admittance

%0.044 0.035 0.030

%servo controller gains:
Kp = 200 %200 %200 %20000  %200
Kv = 40 %40 %3000  %40

npts = 1000;

freq_range = logspace(0,2,npts);
real_pts = zeros(size(freq_range));
mag_pts = zeros(size(freq_range));
phase_pts = zeros(size(freq_range));
i=sqrt(-1);

%compute the open-loop admittance of the uncontrolled robot
freq_range = logspace(-1,4,npts);
real_pts = zeros(size(freq_range));
Ydes_mag_pts = zeros(size(freq_range));
Ydes_phase_pts = zeros(size(freq_range));
Yol_mag_pts = zeros(size(freq_range));
Yol_phase_pts = zeros(size(freq_range));
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[1,0] +Kt_mat/s; % +Fdbk_mat;   
   B_plant = [0;1];
   v_vec = inv(Z_plant)*B_plant;
    v2 =[0,1]* v_vec;    
   Yol_mag_pts(k) = abs(v2);
   Yol_phase_pts(k) = atan2(imag(v2),real(v2));       
    
  end
figure(1)
clf
subplot(2,1,1)
loglog(freq_range, Yol_mag_pts,'r')
ylabel('Y magnitude')
grid on
hold on
title('open-loop robot admittance')
subplot(2,1,2)
semilogx(freq_range,Yol_phase_pts,'r')
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'k')  


%compute and plot Ydes in the frequency domain
%freq_range = logspace(-1,4,npts);
freq_range = logspace(0,4,npts);
real_pts = zeros(size(freq_range));
Ydes_mag_pts = zeros(size(freq_range));
Ydes_phase_pts = zeros(size(freq_range));

for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   Ydes = s/(Bdes*s+Kdes);
   Ydes_mag_pts(k) = abs(Ydes);
   Ydes_phase_pts(k) = atan2(imag(Ydes),real(Ydes));
end

figure(2)
clf
subplot(2,1,1)
loglog(freq_range, Ydes_mag_pts,'b')
ylabel('Y magnitude')
grid on
hold on
title('Ydes')


subplot(2,1,2)
semilogx(freq_range,Ydes_phase_pts,'b')
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on


%compute the closed-loop admittance:

Ycl_mag_pts = zeros(size(freq_range));
Ycl_phase_pts = zeros(size(freq_range));
Ycl_pts = zeros(size(freq_range));
passive=true;
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   %Ydes = s/(Bdes*s+Kdes); or simply 1/Bdes if Kdes=0
    %open-loop plant is:
    %Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[0,1] +Kt_mat/s;
    %Z_plant*v = [1;0]u + [0;1]Fe
   Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[1,0] +Kt_mat/s;    

   %servo_fdbk_mat = [Kp/s+Kv,0;0,0];
  
   %u = [-Kp/s -Kv,0]v +[Kp,0]*x_cmd
   %x_cmd = (1/s)*Y_des*Fe
   Ydes = s/(Bdes*s+Kdes);
   Gf = Kp*Ydes/s; %at Y_des = 1/Bdes
   %Gv = 1/(s^2)*(Kp*Kdes/Bdes)+(Kv + Kp/s);
   Gv = (Kp/s + Kv);
   % simply Gv = (Kv + Kp/s) for Kdes=0
   servo_fdbk_mat = [1;0]*(Gv)*[1,0];
   %more generally:
   %Gv = (exp(-s*T_D)/(s^2))*(Kp)*J_inv*inv(Bvirt)*(Kvirt)*J + (Kv + Kp/s);

    LHS_mat= Z_plant + servo_fdbk_mat;
    RHS_vec = [Gf;1];    %these terms multiply Fe on the right-hand side of the eqns
    
    %LHS_mat*v = RHS_vec*Fe
    %    Y_cl = [J,0]*inv(Z_sys+(gamma_m*Gv*gamma_m'))*[Gf;1]*J';

    v_vec = inv(LHS_mat)*RHS_vec;
    v2 =[0,1]* v_vec;    
    Ycl_pts(k)= v2;
   Ycl_mag_pts(k) = abs(v2);
   Ycl_phase_pts(k) = atan2(imag(v2),real(v2));      
   if ((Ycl_phase_pts(k)<-pi/2) && (passive))
     passive=false;
     phase_crossover = Ycl_phase_pts(k)
     w_crossover = w
     Ycl_mag_crossover = Ycl_mag_pts(k)
   end
end


figure(3)
clf
subplot(2,1,1)
loglog(freq_range, Ydes_mag_pts,'b','linewidth',2)
ylabel('Y magnitude')
grid on
hold on
title('Ydes')


subplot(2,1,2)
semilogx(freq_range,Ydes_phase_pts,'b','linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on


subplot(2,1,1)
loglog(freq_range, Ycl_mag_pts,'r','linewidth',2)
ylabel('Y magnitude')
grid on
hold on
title('robot admittance: des (b), realized (r)','linewidth',2)


subplot(2,1,2)
semilogx(freq_range,Ycl_phase_pts,'r','linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'k')


%compute impedance of the environment
Ze_mag_pts = zeros(size(freq_range));
Ze_phase_pts = zeros(size(freq_range));
Ze_pts = zeros(size(freq_range));

for k=1:npts
  w=freq_range(k);
    s = 1i*w;
    Ze = Kenv/s;
    Ze_pts(k)=Ze;
   Ze_mag_pts(k) = abs(Ze);
   Ze_phase_pts(k) = atan2(imag(Ze),real(Ze));   
end

figure(4)
clf
subplot(2,1,1)
title('magnitude of Zenv vs freq')
%plot(freq_range,mag_pts)
loglog(freq_range, Ze_mag_pts,'linewidth',2)
ylabel('Zenv magnitude')
grid on
title('environment impedance')

subplot(2,1,2)
title('Zenv phase')
%plot(freq_range,phase_pts)
semilogx(freq_range,Ze_phase_pts,'linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
hold on 
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'r')
grid on

%compute Ycl*Zenv:
%freq_range = logspace(1,2,npts);
YrZe_mag_pts = zeros(size(freq_range));
YrZe_phase_pts = zeros(size(freq_range));
passive=true;
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   YclZenv = Ycl_pts(k)*Ze_pts(k);
   YrZe_mag_pts(k) = abs(YclZenv);
   YrZe_phase_pts(k) = atan2(imag(YclZenv),real(YclZenv));
   
   if ((YrZe_phase_pts(k)<-pi) && (passive))
     passive=false;
     YrZe_phase_crossover = YrZe_phase_pts(k)
     w_crossover = w
     YrZe_mag_crossover = YrZe_mag_pts(k)
   end
end
figure(5)
clf
subplot(2,1,1)

%plot(freq_range,mag_pts)
loglog(freq_range, YrZe_mag_pts,'linewidth',2)
ylabel(' magnitude')
grid on
title("Y of robot times Z of environment")

subplot(2,1,2)
title(' phase')
%plot(freq_range,phase_pts)
semilogx(freq_range,YrZe_phase_pts,'linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
hold on 
plot([freq_range(1),freq_range(npts)],[-pi,-pi],'r')
grid on
%title('environment impedance time Ycl')

%display('hit enter to do simu')
%pause
%simulate the controlled system, no time delay

x1=0.1 %non-zero initial condition
x2=0.1
v1=0
v2=0
x_cmd=0;
v_cmd=0;

dt_sim = 0.0001;
time_vec = 0:dt_sim:5;
[dummy npts_sim]=size(time_vec)

x2_hist=zeros(1,npts_sim);
v2_hist = zeros(1,npts_sim);
Fe_hist = zeros(1,npts_sim);
u_hist = zeros(1,npts_sim);
for k=1:npts_sim
  Fe = -Kenv*x2;  %for environment of a pure spring w/ stiffness Kenv
  v_cmd = (1/Bdes)*(Fe-Kdes*x1); %compute velocity that would be consistent w/ Ydes
  x_cmd=x_cmd+v_cmd*dt_sim; %and position consistent w/ Ydes
  
  %servo control:
  u=Kp*(x_cmd-x1)-Kv*v1; %send xdes to the servo controller

  x2_hist(k)=x2;
  Fe_hist(k)=Fe;
  u_hist(k)=u;
  
  %Euler 1-step simulation:
  F_trans = Bt*(v2-v1)+Kt*(x2-x1);
  a1 = (1/m1)*(F_trans+u -B_actuator*v1);
  a2 = (1/m2)*(-F_trans+Fe);
  
  v1_old = v1;
  v1=v1+a1*dt_sim; %Euler 1-step numerical integration
  x1=x1+0.5*(v1+v1_old)*dt_sim;
  v2_old = v2;
  v2=v2+a2*dt_sim; 
  x2=x2+0.5*(v2+v2_old)*dt_sim;  
  
  end

  figure(6)
  clf
  subplot(3,1,1)
  plot(time_vec,x2_hist,'b')
  title('compliance controller response: x2(t)')
  xlabel('time (sec)')
  ylabel('x (m)')
  grid on
  subplot(3,1,2)
  plot(time_vec,Fe_hist,'b')
  title('compliance controller response: Fe(t)')
  xlabel('time (sec)')
  ylabel('F (N)')
  grid on
  subplot(3,1,3)
  plot(time_vec,u_hist,'b')
  title('compliance controller response: u(t)')
  xlabel('time (sec)')
  ylabel('F (N)')
  grid on


%passivity analysis of a 1-DOF system
clear all
m1 = 1 %system model: a 1kg mass proximal to actuator
m2 = 1 %and 1kg mass distal from motor
Kt = 1000 %stiffness of transmission
Bt = 50 %damping of transmission
M=[m1,0;0,m2]
omega_n = sqrt(2*Kt/(m1+m2))

Bt_mat = [Bt,-Bt;-Bt,Bt];
Kt_mat = [Kt,-Kt;-Kt,Kt];
B_actuator = 10; %actuator has lots of friction; want to mask this with feedback



Kenv= 200 %23 %100 %120 %200000 %environment model: a spring of this stiffness, N/m

%desired impedance of Bdes and Kdes
%LOWER BDES IS BETTER, implies more responsive closed-loop admittance of the robot
l = 4.5 ;

h = 5.8 ;
Bdes = ( l + h)/2%damping of desired admittance
% Bdes = l;
% Bdes = h;
Kdes = 0; %10; %20; %100 %stiffness of desired admittance

%0.044 0.035 0.030

%servo controller gains:
Kp = 200%200 %200 %20000  %200
Kv = 40 %40 %3000  %40

%0.045 0.054
% 0.035
npts = 1000;

freq_range = logspace(0,2,npts);
real_pts = zeros(size(freq_range));
mag_pts = zeros(size(freq_range));
phase_pts = zeros(size(freq_range));
i=sqrt(-1);

%compute the open-loop admittance of the uncontrolled robot
freq_range = logspace(-1,4,npts);
real_pts = zeros(size(freq_range));
Ydes_mag_pts = zeros(size(freq_range));
Ydes_phase_pts = zeros(size(freq_range));
Yol_mag_pts = zeros(size(freq_range));
Yol_phase_pts = zeros(size(freq_range));
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[1,0] +Kt_mat/s; % +Fdbk_mat;   
   B_plant = [0;1];
   v_vec = inv(Z_plant)*B_plant;
    v2 =[0,1]* v_vec;    
   Yol_mag_pts(k) = abs(v2);
   Yol_phase_pts(k) = atan2(imag(v2),real(v2));       
    
  end
figure(1)
clf
subplot(2,1,1)
loglog(freq_range, Yol_mag_pts,'r')
ylabel('Y magnitude')
grid on
hold on
title('open-loop robot admittance')
subplot(2,1,2)
semilogx(freq_range,Yol_phase_pts,'r')
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'k')  


%compute and plot Ydes in the frequency domain
%freq_range = logspace(-1,4,npts);
freq_range = logspace(0,4,npts);
real_pts = zeros(size(freq_range));
Ydes_mag_pts = zeros(size(freq_range));
Ydes_phase_pts = zeros(size(freq_range));

for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   Ydes = s/(Bdes*s+Kdes);
   Ydes_mag_pts(k) = abs(Ydes);
   Ydes_phase_pts(k) = atan2(imag(Ydes),real(Ydes));
end

figure(2)
clf
subplot(2,1,1)
loglog(freq_range, Ydes_mag_pts,'b')
ylabel('Y magnitude')
grid on
hold on
title('Ydes')


subplot(2,1,2)
semilogx(freq_range,Ydes_phase_pts,'b')
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on


%compute the closed-loop admittance:

Ycl_mag_pts = zeros(size(freq_range));
Ycl_phase_pts = zeros(size(freq_range));
Ycl_pts = zeros(size(freq_range));
passive=true;
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   %Ydes = s/(Bdes*s+Kdes); or simply 1/Bdes if Kdes=0
    %open-loop plant is:
    %Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[0,1] +Kt_mat/s;
    %Z_plant*v = [1;0]u + [0;1]Fe
   Z_plant = M*s +Bt_mat + [1;0]*B_actuator*[1,0] +Kt_mat/s;    

   %servo_fdbk_mat = [Kp/s+Kv,0;0,0];
  
   %u = [-Kp/s -Kv,0]v +[Kp,0]*x_cmd
   %x_cmd = (1/s)*Y_des*Fe
   Ydes = s/(Bdes*s+Kdes);
   Gf = Kp*Ydes/s; %at Y_des = 1/Bdes
   %Gv = 1/(s^2)*(Kp*Kdes/Bdes)+(Kv + Kp/s);
   Gv = (Kp/s + Kv);
   % simply Gv = (Kv + Kp/s) for Kdes=0
   servo_fdbk_mat = [1;0]*(Gv)*[1,0];
   %more generally:
   %Gv = (exp(-s*T_D)/(s^2))*(Kp)*J_inv*inv(Bvirt)*(Kvirt)*J + (Kv + Kp/s);

    LHS_mat= Z_plant + servo_fdbk_mat;
    RHS_vec = [Gf;1];    %these terms multiply Fe on the right-hand side of the eqns
    
    %LHS_mat*v = RHS_vec*Fe
    %    Y_cl = [J,0]*inv(Z_sys+(gamma_m*Gv*gamma_m'))*[Gf;1]*J';

    v_vec = inv(LHS_mat)*RHS_vec;
    v2 =[0,1]* v_vec;    
    Ycl_pts(k)= v2;
   Ycl_mag_pts(k) = abs(v2);
   Ycl_phase_pts(k) = atan2(imag(v2),real(v2));      
   if ((Ycl_phase_pts(k)<-pi/2) && (passive))
     passive=false;
     phase_crossover = Ycl_phase_pts(k)
     w_crossover = w
     Ycl_mag_crossover = Ycl_mag_pts(k)
   end
end


figure(3)
clf
subplot(2,1,1)
loglog(freq_range, Ydes_mag_pts,'b','linewidth',2)
ylabel('Y magnitude')
grid on
hold on
title('Ydes')


subplot(2,1,2)
semilogx(freq_range,Ydes_phase_pts,'b','linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on


subplot(2,1,1)
loglog(freq_range, Ycl_mag_pts,'r','linewidth',2)
ylabel('Y magnitude')
grid on
hold on
title('robot admittance: des (b), realized (r)','linewidth',2)


subplot(2,1,2)
semilogx(freq_range,Ycl_phase_pts,'r','linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
grid on
hold on
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'k')


%compute impedance of the environment
Ze_mag_pts = zeros(size(freq_range));
Ze_phase_pts = zeros(size(freq_range));
Ze_pts = zeros(size(freq_range));

for k=1:npts
  w=freq_range(k);
    s = 1i*w;
    Ze = Kenv/s;
    Ze_pts(k)=Ze;
   Ze_mag_pts(k) = abs(Ze);
   Ze_phase_pts(k) = atan2(imag(Ze),real(Ze));   
end

figure(4)
clf
subplot(2,1,1)
title('magnitude of Zenv vs freq')
%plot(freq_range,mag_pts)
loglog(freq_range, Ze_mag_pts,'linewidth',2)
ylabel('Zenv magnitude')
grid on
title('environment impedance')

subplot(2,1,2)
title('Zenv phase')
%plot(freq_range,phase_pts)
semilogx(freq_range,Ze_phase_pts,'linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
hold on 
plot([freq_range(1),freq_range(npts)],[-pi/2,-pi/2],'r')
grid on

%compute Ycl*Zenv:
%freq_range = logspace(1,2,npts);
YrZe_mag_pts = zeros(size(freq_range));
YrZe_phase_pts = zeros(size(freq_range));
passive=true;
for k=1:npts
  w=freq_range(k);
    s = 1i*w;
   YclZenv = Ycl_pts(k)*Ze_pts(k);
   YrZe_mag_pts(k) = abs(YclZenv);
   YrZe_phase_pts(k) = atan2(imag(YclZenv),real(YclZenv));
   
   if ((YrZe_phase_pts(k)<-pi) && (passive))
     passive=false;
     YrZe_phase_crossover = YrZe_phase_pts(k)
     w_crossover = w
     YrZe_mag_crossover = YrZe_mag_pts(k)
   end
end
figure(5)
clf
subplot(2,1,1)

%plot(freq_range,mag_pts)
loglog(freq_range, YrZe_mag_pts,'linewidth',2)
ylabel(' magnitude')
grid on
title("Y of robot times Z of environment")

subplot(2,1,2)
title(' phase')
%plot(freq_range,phase_pts)
semilogx(freq_range,YrZe_phase_pts,'linewidth',2)
xlabel('freq (rad/sec)')
ylabel('phase (rad)')
hold on 
plot([freq_range(1),freq_range(npts)],[-pi,-pi],'r')
grid on
%title('environment impedance time Ycl')

%display('hit enter to do simu')
%pause
%simulate the controlled system, no time delay

x1=0.1 %non-zero initial condition
x2=0.1
v1=0
v2=0
x_cmd=0;
v_cmd=0;

dt_sim = 0.0001;
time_vec = 0:dt_sim:5;
[dummy npts_sim]=size(time_vec)

x2_hist=zeros(1,npts_sim);
v2_hist = zeros(1,npts_sim);
Fe_hist = zeros(1,npts_sim);
u_hist = zeros(1,npts_sim);
for k=1:npts_sim
  Fe = -Kenv*x2;  %for environment of a pure spring w/ stiffness Kenv
  v_cmd = (1/Bdes)*(Fe-Kdes*x1); %compute velocity that would be consistent w/ Ydes
  x_cmd=x_cmd+v_cmd*dt_sim; %and position consistent w/ Ydes
  
  %servo control:
  u=Kp*(x_cmd-x1)-Kv*v1; %send xdes to the servo controller

  x2_hist(k)=x2;
  Fe_hist(k)=Fe;
  u_hist(k)=u;
  
  %Euler 1-step simulation:
  F_trans = Bt*(v2-v1)+Kt*(x2-x1);
  a1 = (1/m1)*(F_trans+u -B_actuator*v1);
  a2 = (1/m2)*(-F_trans+Fe);
  
  v1_old = v1;
  v1=v1+a1*dt_sim; %Euler 1-step numerical integration
  x1=x1+0.5*(v1+v1_old)*dt_sim;
  v2_old = v2;
  v2=v2+a2*dt_sim; 
  x2=x2+0.5*(v2+v2_old)*dt_sim;  
  
  end

  figure(6)
  clf
  subplot(3,1,1)
  plot(time_vec,x2_hist,'b')
  title('compliance controller response: x2(t)')
  xlabel('time (sec)')
  ylabel('x (m)')
  grid on
  subplot(3,1,2)
  plot(time_vec,Fe_hist,'b')
  title('compliance controller response: Fe(t)')
  xlabel('time (sec)')
  ylabel('F (N)')
  grid on
  subplot(3,1,3)
  plot(time_vec,u_hist,'b')
  title('compliance controller response: u(t)')
  xlabel('time (sec)')
  ylabel('F (N)')
  grid on


