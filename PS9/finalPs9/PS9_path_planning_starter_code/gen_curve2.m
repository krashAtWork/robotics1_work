 wrap_12=[pi;0];
 path_end=[0.5;0.3];
intmdt_pts = [[2.358; 1.258], [1.958; pi],[1.958; -pi;],[1.5548; -2.242],[1.5548; -1.542]];
part3 = [intmdt_pts(:,3), intmdt_pts(:,4),intmdt_pts(:,5), path_end];
part3_2 = [[1;-1.2], [0.7; -0.7] , [0.6; 0.1], path_end]
% x3 = part3(1,:)
% y3 = part3(2,:)
x3_2 = part3_2(1,:)
y3_2 = part3_2(2,:)
[fitresult, gof] = createFit3(x3_2, y3_2)
    
%        x = pi:-0.1:1.958
%        a =       129.1  
%        b =      -5.517  
%        y = a*x.^b
       
       
       
       
% figure
% plot(x,y,'b*')
% hold on
% plot(part2(1,:), part2(2,:),'r','linewidth',2)
% grid on


% (1,-1.2), (0.7, -0.7) and (0.6, 0.1)
