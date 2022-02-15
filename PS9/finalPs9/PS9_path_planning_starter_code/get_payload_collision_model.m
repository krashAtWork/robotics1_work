
function [payload_ctr_pts,payload_radii] = get_payload_collision_model (q_vec,a_vec)
    hand_pt = [a_vec(1)*cos(q_vec(1))+a_vec(2)*cos(q_vec(1)+q_vec(2)); a_vec(1)*sin(q_vec(1))+a_vec(2)*sin(q_vec(1)+q_vec(2))];
    elbow_pt = [a_vec(1)*cos(q_vec(1));a_vec(1)*sin(q_vec(1))];
% transposing for convenience
    hand_pt = hand_pt';
    elbow_pt = elbow_pt';
  payload_length = 1.2;
  payload_width = 0.1;
  %% modeling the above two lengths into circles
  payload_circle_radius = payload_width/2;
  n_circles_payload = round(payload_length/payload_circle_radius);
%     n_circles_payload =3;
  payload_ctr_pts=[ hand_pt']; %FIX ME!
  payload_radii=[ payload_circle_radius]; %fill this with payload_width/2 for each sample point
  slope_forearm = (hand_pt(2) - elbow_pt(2))/ (hand_pt(1) - elbow_pt(1)) ;%(Y2 - Y1) / (X2 - X1)
  slope_payload = -1/slope_forearm;
  
  %% LOOP THE SAME FOR TOTAL CIRCLES/2
  
  one_side = floor(n_circles_payload/2);
  d = payload_width/2;
  x0 = hand_pt(1);
  y0 = hand_pt(2);
  for i=1: one_side
      
      x = d /sqrt(1 + slope_payload^2) + x0;
      y = (slope_payload*(x - x0)) + y0;
      new_pt = [x;y];
      x0 = x;
      y0 = y;
     
      payload_ctr_pts = [payload_ctr_pts , new_pt];
      payload_radii = [payload_radii, payload_circle_radius];
  end
  
  x0 = hand_pt(1);
  y0 = hand_pt(2);
  
    for i=1: one_side 
           
      x = -d /sqrt(1 + slope_payload^2) + x0;
      y = (slope_payload*(x - x0)) + y0;
      new_pt = [x;y];
      x0 = x;
      y0 = y;
     
      payload_ctr_pts = [payload_ctr_pts , new_pt];
      payload_radii = [payload_radii, payload_circle_radius];
    end
  
  
  
  
 
end
