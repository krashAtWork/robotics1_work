function [A] = find_A_from_DH(a, d, alpha, theta)
%     a = dh_row(1,1);
%     d = dh_row(1,2);
%     alpha = dh_row(1,3);
%     theta = dh_row(1,4);
    
%     size(dh_row);
    A = eye(4); % EYE(N) is the N-by-N identity matrix.
    %row 1
    A(1,1) = cos(theta);
    A(1,2) = -sin(theta)*cos(alpha);
    A(1,3) = sin(theta)* sin(alpha);
    A(1,4) = a* cos(theta);
    % row 2
    A(2,1) = sin(theta);
    A(2,2) = cos(theta)*cos(alpha);
    A(2,3) = -cos(theta)* sin(alpha);
    A(2,4) = a* sin(theta);
    % row 3
    A(3,1) = 0;
    A(3,2) = sin(alpha);
    A(3,3) = cos(alpha);
    A(3,4) = d ;
    % row 4
    A(4,1:3) = 0;  
%    A(4,4) = d ;
end  