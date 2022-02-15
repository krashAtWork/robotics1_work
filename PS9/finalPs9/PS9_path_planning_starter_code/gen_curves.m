   wrap_12=[pi;0];
intmdt_pts = [[2.358; 1.258], [1.958; pi],[1.958; -pi;],[1.5548; -2.242],[1.5548; -1.542]];

part2 = [wrap_12, intmdt_pts(:,1),intmdt_pts(:,2) ];
    
       x = pi:-0.1:1.958
       a =       129.1  
       b =      -5.517  
       y = a*x.^b
       
       
       
       
figure
plot(x,y,'b*')
hold on
plot(part2(1,:), part2(2,:),'r','linewidth',2)
grid on


%  fitresult(x) = a*x^b
%      Coefficients (with 95% confidence bounds):
%        a =       129.1  (-1219, 1477)
%        b =      -5.517  (-20.47, 9.434)