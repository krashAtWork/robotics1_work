%function to define environmental obstacles as a collection of circles
%return a matrix of centerpoints and corresponding radii
function [obstacle_ctr_pts,obstacle_radii]= get_environment_model ()
obstacle_ctr_pts=[1;0];
obstacle_radii=[0.1];

obstacle_ctr_pts=[obstacle_ctr_pts,[-0.8;-0.8]];
obstacle_radii=[obstacle_radii,0.1];

obstacle_ctr_pts=[obstacle_ctr_pts,[-1;1]];
obstacle_radii=[obstacle_radii,0.1];

obstacle_ctr_pts=[obstacle_ctr_pts,[0.5;1.4]];
obstacle_radii=[obstacle_radii,0.2];
end


%% OBSERVE THE CODE :

% DEFINE A SET AND ADD THE NEXT POINT TO THE SAME SET.
