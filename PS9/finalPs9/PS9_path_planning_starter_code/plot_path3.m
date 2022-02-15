%this contains a non-viable path.  FIX ME!!

function  plot_path3 (path_start,path_end,a_vec)

%% code for continuous path, can be broken down into 3 parts
via_1=[-pi;0];
intmdt_pts = [ [1.958; pi],[1.958; -pi;],[1.5548; -2.242],[1.5548; -1.542]];
wrap_12=[pi;0];
part1 = [path_start, via_1];
part2 = [wrap_12, intmdt_pts(:,1)];
part3 = [intmdt_pts(:,2), intmdt_pts(:,3),intmdt_pts(:,4), path_end];
dq_size = 0.1;
[obstacle_ctr_pts,obstacle_radii] = get_environment_model;
fignum=3;
%% make a continuous valid path



 %% WHAT DO THESE MEAN WRAP_12 IS THE NEGATIVE VIA_1
figure(2)
hold on
 
 %% MARKS THE START AND END POINTS
 plot(path_start(1),path_start(2),'g.',"markersize",50)
 plot(path_end(1),path_end(2),'g.',"markersize",50)
[dummy, imp] = size(intmdt_pts);
for i = 1: imp
   plot(intmdt_pts(1,i),intmdt_pts(2,i),'r.',"markersize",25)
end
 
%% Part 1

dist1 = norm(path_start - via_1);%% CALCULATING DISTANCE BETWEEN TWO POINTS\

npts = round(dist1/dq_size); %% BREAKING A LINE SEGMENT INTO SMALLER UNITS

figure(fignum);
clf

plot_circles(obstacle_ctr_pts,obstacle_radii,fignum);
fignum=3;
figure(fignum)
for i=0:npts %% FOR EACH POINT ALONG THE ABOVE DISTANCE 
  figure(3)
clf
plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
hold on
axis([-3,3,-3,3])
grid on
axis('square')
  q_vec = path_start+(via_1-path_start)*i/npts;
  [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
[payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);

arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
arm_model_radii=[arm_model_radii,payload_radii];
plot_circles(arm_ctr_pts,arm_model_radii,fignum)
axis([-3,3,-3,3])
 figure(2)
 hold on
 plot(q_vec(1),q_vec(2),'k.',"markersize",30)
pause
end

%% PART2 - CURVED LINE USING 3 POINTS

 x = pi:-0.1:1.958
 a =       129.1  
 b =      -5.517  
 y = a*x.^b
 
[ dummy,npts2] = size(x)
 for i= 1 : npts2
     disp('here')
      figure(3)
      clf
      plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
      hold on
      axis([-3,3,-3,3])
       grid on
      axis('square')
      q_vec= [x(i);y(i)]
      [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
       arm_model_radii=[arm_model_radii,payload_radii];
      plot_circles(arm_ctr_pts,arm_model_radii,fignum)
      figure(2)
        hold on

 plot(q_vec(1),q_vec(2),'k.',"markersize",30)
pause
      
 end
       
       
% hold on;      
% figure(3)
% clf
% plot(x,y,'g*','linewidth',20)
% hold on
% plot(part2(1,:), part2(2,:),'r','linewidth',2)
% grid on


%% PART3 - SINGLE LINE
% dist2 = norm(wrap_12 - intmdt_pts(:,1));
% npts = round(dist2/dq_size);
% 
% for i=0:npts
%   figure(3)
% clf
% plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
% hold on
% axis([-3,3,-3,3])
% grid on
% axis('square')  
%   q_vec = wrap_12+(intmdt_pts(:,1)-wrap_12)*i/npts;
%   [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
% [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
% 
% arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
% arm_model_radii=[arm_model_radii,payload_radii];
% plot_circles(arm_ctr_pts,arm_model_radii,fignum)
%  figure(2)
%   hold on
% 
%  plot(q_vec(1),q_vec(2),'k.',"markersize",30)
% pause
% end

%% PART3 - MULTIPLE LINES

%% Part1 - Jos
% part3_1 = [[1.958; -3.142], [1.658; -2.842], [1.555; -2.242]];
part3_1 = [[1.958; -3.142], [1.658; -2.842], [1.552; -2.242]];

% x3_1 = part3_1(1,:);
% y3_1 = part3_1(2,:);
    
%        x = 1.958:-0.1:1.555
%        p1 =       11.97
%        p2 =       -44.3
%        p3 =       37.69 
%        y = p1*x.^2 + p2*x + p3
x = 1.958:-0.1:1.552
p1 =       11.48
p2 =      -42.51
p3 =       36.08
y = p1*x.^2 + p2*x + p3;
 [ dummy,npts3_1] = size(x) ;
 
 for i= 1 : npts3_1
     disp('here')
      figure(3)
      clf
      plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
      hold on
      axis([-3,3,-3,3])
       grid on
      axis('square')
      q_vec= [x(i);y(i)]
      [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
       arm_model_radii=[arm_model_radii,payload_radii];
      plot_circles(arm_ctr_pts,arm_model_radii,fignum)
      figure(2)
        hold on

 plot(q_vec(1),q_vec(2),'k.',"markersize",30)
pause
      
 end
 

%% Part2
part3_2 = [[1.552; -2.242], [1.658; -1.842], [1.555; -1.142]];
[dummy, imp] = size(part3_2);
% imp = 2;
dist3_2 = zeros(1, imp);
npts3_2 = zeros(1, imp);


for j= 1:(imp-1)
    dist3_2(j) = norm(part3_2(:,j) - part3_2(:,j+1)); 
    npts3_2(j) = round(dist3_2(j)/dq_size);
    for i=0: npts3_2(j)
    figure(3)
    clf
    plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
    hold on
    axis([-3,3,-3,3])
    grid on
    axis('square')
%     q_vec = wrap_12+(intmdt_pts(:,1)-wrap_12)*i/npts;
    q_vec = part3_2(:,j)+(part3_2(:,j+1)- part3_2(:,j))*i/npts3_2(j);
    [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
    [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);

    arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
    arm_model_radii=[arm_model_radii,payload_radii];
    plot_circles(arm_ctr_pts,arm_model_radii,fignum)
    figure(2)
    hold on

    plot(q_vec(1),q_vec(2),'k.',"markersize",30)
    pause
    
    end
    
end
% x = 1.552:0.05:1.658
% p1 =      -3523
% p2 =  1.131e+04
% p3 =       -9074
% y = p1*x.^2 + p2*x + p3
%  [dummy,npts3_2] = size(x)
%  
%  for i= 1 : npts3_2
%      disp('here')
%       figure(3)
%       clf
%       plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
%       hold on
%       axis([-3,3,-3,3])
%        grid on
%       axis('square')
%       q_vec= [x(i);y(i)]
%       [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
%       [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
%       [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
%       arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
%        arm_model_radii=[arm_model_radii,payload_radii];
%       plot_circles(arm_ctr_pts,arm_model_radii,fignum)
%       figure(2)
%         hold on
% 
%  plot(q_vec(1),q_vec(2),'k.',"markersize",30)
% pause
%       
%  end

%% Part3 - 
path_end=[0.5;0.3];     
part3_3 = [[1;-1.2], [0.7; -0.7] , [0.6; 0.1], path_end]
xnw1 = 1:-0.05:0.6;

disp("kkk")
[ dummy,npts3_3] = size(xnw1)
p1 =       4.945 ; 
p2 =      -10.63 ; 
p3 =        4.47 ; 
ynw1 = p1*xnw1.^2 + p2*xnw1 + p3
for i= 1 : npts3_3
     disp('here')
     i
      figure(3)
      clf
      plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
      hold on
      axis([-3,3,-3,3])
       grid on
      axis('square')
      q_vec= [xnw1(i);ynw1(i)]
      [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
      arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
       arm_model_radii=[arm_model_radii,payload_radii];
      plot_circles(arm_ctr_pts,arm_model_radii,fignum)
      figure(2)
        hold on

 plot(q_vec(1),q_vec(2),'k.',"markersize",30)
pause
      
 end
     
%     figure
%     plot(xnw1,ynw1,'b*')
%     hold on
%     plot(part3_2(1,:), part3_2(2,:),'r','linewidth',2)
%     grid on


% make a loop of the above process:
% [dummy, imp] = size(part3);
% % imp = 2;
% dist3 = zeros(1, imp);
% npts3 = zeros(1, imp);
% 
% 
% for j= 1:(imp-1)
%     dist3(j) = norm(part3(:,j) - part3(:,j+1)); 
%     npts3(j) = round(dist3(j)/dq_size);
%     for i=0: npts3(j)
%     figure(3)
%     clf
%     plot_circles(obstacle_ctr_pts,obstacle_radii,fignum)
%     hold on
%     axis([-3,3,-3,3])
%     grid on
%     axis('square')
% %     q_vec = wrap_12+(intmdt_pts(:,1)-wrap_12)*i/npts;
%     q_vec = part3(:,j)+(part3(:,j+1)- part3(:,j))*i/npts3(j);
%     [arm_ctr_pts,arm_model_radii] = get_arm_collision_model(q_vec,a_vec);
%     [payload_ctr_pts,payload_radii] = get_payload_collision_model(q_vec,a_vec);
% 
%     arm_ctr_pts=[arm_ctr_pts,payload_ctr_pts];
%     arm_model_radii=[arm_model_radii,payload_radii];
%     plot_circles(arm_ctr_pts,arm_model_radii,fignum)
%     figure(2)
%     hold on
% 
%     plot(q_vec(1),q_vec(2),'k.',"markersize",30)
%     pause
%     
%     end
%     
% end

end
