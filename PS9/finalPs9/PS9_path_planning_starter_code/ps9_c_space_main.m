%start of program for C-space computation
%compute safe/unsafe poses by looking at radii between points on robot links and
%obstacles in the environment
clc
a_vec=[1,1]; %DH link lengths

%first describe the environment obstacles as a collection of circles specified
% in terms of centers and radii
[obstacle_ctr_pts,obstacle_radii] = get_environment_model;
fignum=1;
figure(fignum);
clf %CLEAR CURRENT FIGURES
plot_circles(obstacle_ctr_pts,obstacle_radii,fignum);


%the arm collision model will be described as a collection of circles
%at specified sample points along each link, together with respective radii
%these are a function of arm angles, q_vec
 path_start=[-2.8;0.4];
 path_end=[0.5;0.3];
q_vec = path_start;
[arm_ctr_pts,arm_model_radii ] = get_arm_collision_model(q_vec,a_vec); %% WHAT DOES THIS FUNCTION DO ?
[payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
% payload_ctr_pts=[]; %FIX ME!
%   payload_radii=[]
%% Just include the payload orientation and ,
% i think a_vec = length of payload, It would be easy if the payload was
%%%%% one sided on the robot, so this will be a little challenging

%[payload_ctr_pts,payload_radii] = get_payload_collision_model_soln(q_vec,a_vec);

arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
arm_model_radii=[arm_model_radii,payload_radii];
plot_circles(arm_ctr_pts, arm_model_radii, fignum);
axis([-3,3,-3,3])
axis('square') %% WHAT DOES THIS DO ?
grid on
title('desired start pose')
xlabel('x (m)')
ylabel('y (m)')

display('desired start pose; hit enter to display payload')
pause
hold on
size(payload_ctr_pts)
plot_circles(payload_ctr_pts,payload_radii,fignum)
title('desired start pose with payload')
display('hit enter to display desired goal pose')
pause

figure(1)
clf

q_vec = path_end;
[arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
[payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
% payload_ctr_pts=[]; %FIX ME!
%   payload_radii=[]
%[payload_ctr_pts,payload_radii] = get_payload_collision_model_soln(q_vec,a_vec);
arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts]; %% whY APPENDING
arm_model_radii=[arm_model_radii,payload_radii];
plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
hold on
plot_circles(arm_ctr_pts,arm_model_radii,fignum)
axis([-3,3,-3,3])
axis('square')
grid on
title('desired goal pose')
xlabel('x (m)')
ylabel('y (m)')

display('computing C-space')
figure(2)
clf
hold on
axis([-pi,pi,-pi,pi])
axis('square')
for q1=-pi:0.1:pi
  for q2=-pi:0.1:pi
    q_vec=[q1;q2];
    [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec); %% pose-> find arm circle centers and radius->  to show in plot, to check for colllision
     [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
%     payload_ctr_pts=[]; %FIX ME!
%   payload_radii=[]
%     % [payload_ctr_pts,payload_radii] = get_payload_collision_model_soln(q_vec,a_vec);
     
    arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
    arm_model_radii=[arm_model_radii,payload_radii];
    if (is_collision(arm_ctr_pts,arm_model_radii,obstacle_ctr_pts,obstacle_radii)) 
      plot(q1,q2,'r.',"markersize",30);  %'k.',"markersize",30
    else
      plot(q1,q2,'bx');
    end
    
  end
end
title('C-space')
xlabel('q1 (rad)')
ylabel('q2 (rad)')
% 
% display('displaying path incrementally; hit return to increment')
% 
% path is embedded in plot_path

plot_path(path_start,path_end,a_vec)
% %plot_path_soln(path_start,path_end,a_vec)
% 
