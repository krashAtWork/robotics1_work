 path_end=[0.5;0.3];     
part3_2 = [[1;-1.2], [0.7; -0.7] , [0.6; 0.1], path_end]
       xnw1 = 1:-0.1:0.6
       p1 =       4.945 ; 
       p2 =      -10.63 ; 
       p3 =        4.47 ; 
     ynw1 = p1*xnw1.^2 + p2*xnw1 + p3
     
    figure
    plot(xnw1,ynw1,'b*')
    hold on
    plot(part3_2(1,:), part3_2(2,:),'r','linewidth',2)
    grid on

% Coefficients (with 95% confidence bounds):
     
       
       
       %%% first you need the points
       