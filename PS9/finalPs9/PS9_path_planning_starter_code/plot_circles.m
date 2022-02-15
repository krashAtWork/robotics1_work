%plot out circles described by centers and radii



function [retval] = plot_circles (ctr_pts,radii,fignum)
  figure(fignum);
  hold on
  theta=0:0.1:2*pi;
  unit_circle=[cos(theta);sin(theta)];

  [dummy,npts]=size(ctr_pts);
  for i=1:npts
    circle=radii(i)*unit_circle + ctr_pts(:,i);
   plot(circle(1,:),circle(2,:),'*');
   end

end
