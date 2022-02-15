%function to saturate values at specified (symmetric) limits
function [x_out] = sat_vec(x_sat, x_vec)
  x_out=x_vec; %default
  [dim,dummy]=size(x_vec);
  x_out=x_vec;
  for i=1:dim
    if x_vec(i)>x_sat(i) %test for positive or negative saturation
      x_out(i)=x_sat(i);
    elseif x_vec(i)< -x_sat(i)
      x_out(i)=-x_sat(i);
   end
 
end