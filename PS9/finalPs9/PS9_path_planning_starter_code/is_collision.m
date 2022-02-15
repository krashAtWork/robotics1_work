

function [collision_bool] = is_collision (arm_ctr_pts,arm_model_radii,obstacle_ctr_pts,obstacle_radii)
  collision_bool=false;
  [dummy,npts_env] = size(obstacle_ctr_pts);
  [dummy,npts_arm] = size(arm_ctr_pts);
  for i_env=1:npts_env
    for i_arm = 1:npts_arm
      dist = norm(arm_ctr_pts(:,i_arm)-obstacle_ctr_pts(:,i_env));
      if (dist<(arm_model_radii(i_arm)+obstacle_radii(i_env)))
        collision_bool=true;
        return;
      end
    end
  end
  

end
