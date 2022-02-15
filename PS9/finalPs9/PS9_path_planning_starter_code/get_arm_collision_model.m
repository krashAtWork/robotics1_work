

function [arm_ctr_pts,arm_model_radii] = get_arm_collision_model (q_vec,a_vec)%% FOR A POSE, AND DIMENSIONS OF THE ROBOT
link1_width = 0.1; % SAME WIDTH
link2_width = 0.1;
link1_circle_radius=link1_width/2; % LINK DIAMETER IS SAME AS LINK WIDTH
link2_circle_radius=link2_width/2;
n_circles_humerus = round(a_vec(1)/link1_circle_radius);% NUMBER OF CIRCLES REQUIRED % ROUND INSTEAD OF FLOOR
samp_dist1 = a_vec(1)/(n_circles_humerus);
elbow_pt = [a_vec(1)*cos(q_vec(1));a_vec(1)*sin(q_vec(1))];
shoulder_pt = [0;0];
hand_pt = [a_vec(1)*cos(q_vec(1))+a_vec(2)*cos(q_vec(1)+q_vec(2)); a_vec(1)*sin(q_vec(1))+a_vec(2)*sin(q_vec(1)+q_vec(2))];
n_circles_forearm= round(a_vec(2)/link2_circle_radius);
samp_dist2 = a_vec(2)/(n_circles_forearm);
arm_ctr_pts=shoulder_pt;
arm_model_radii=[link1_circle_radius];
for isamp=1:n_circles_humerus
  samp_pt = shoulder_pt+(elbow_pt-shoulder_pt)*isamp/n_circles_humerus;
  arm_ctr_pts=[arm_ctr_pts,samp_pt];
  arm_model_radii=[arm_model_radii,link1_circle_radius];
end
for isamp=1:n_circles_forearm
  samp_pt = elbow_pt+(hand_pt-elbow_pt)*isamp/n_circles_forearm;
  arm_ctr_pts=[arm_ctr_pts,samp_pt];
  arm_model_radii=[arm_model_radii,link2_circle_radius];
end
  
end
