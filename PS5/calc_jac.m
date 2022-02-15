function Jac_pos  = calc_jac(A_dqi)
    disp("I am here")
%     Jac = zeros(3,3);
    % orientation Jacobian Parameters: b^ vectors in 0,1 and 2 frames.
    b_cap0 = [0, 0, 1]';
    b_cap1 = A_dqi(1:3,3); %obtain from R1_0 3rd column
    b_cap2 = A_dqi(5:7,3); %obtain from R2_0

    Jac_orient = [b_cap0, b_cap1, b_cap2];


    % positional Jacobian Parametrs:
    % required params
    o4_0 = A_dqi(17:19, 4);
    o1_0 = A_dqi(1:3, 4);% from R1_0 4th column
    o2_0 = A_dqi(5:7, 4);% from R2_0


    J1p = cross(b_cap0 , o4_0); % cross product

    J2p = cross(b_cap1 , (o4_0 - o1_0));
    J3p = cross(b_cap2 , (o4_0 - o2_0));

    Jac_pos = [J1p, J2p, J3p];
   
    
    
end