%function to saturate values at specified (symmetric) limits
function [x_out] = sat(x_sat, x)
  x_out=x; %default
 if x>x_sat %test for positive or negative saturation
   x_out=x_sat;
 elseif x< -x_sat
   x_out=-x_sat;
 end
 
end