function [A_matrices] = fk (a_vec, d_vec, alpha_vec, theta_vec)
    [discard,joints] = size(a_vec) % [1,3];
    % intermediate values
    szAref = [4*joints,4];
    Aref = zeros(szAref);
    for i = 1 : joints
        if i == 1 %%% because of the index values starting from 1, two different conditions for joint 1 and others respectively           
            Aref(i:i+3,:) = find_A_from_DH(a_vec(i),d_vec(i), alpha_vec(i), theta_vec(i));
        else
            Aref((4*(i-1))+1:(4*(i-1))+ 4,:) = find_A_from_DH(a_vec(i),d_vec(i), alpha_vec(i), theta_vec(i));            
        end
    end
    
   A_matrices = zeros(szAref);
    for i = 1: joints
        temp = eye(4);
        if i == 1
            A_matrices(i:i+3,:) = Aref(i:i+3,:)* temp
            temp = A_matrices(i:i+3,:)
        else
            A_matrices((4*(i-1))+1:(4*(i-1))+ 4,:) = temp * Aref((4*(i-1))+1:(4*(i-1))+ 4,:);
            temp = A_matrices((4*(i-1))+1:(4*(i-1))+ 4,:)
        end
    end
    
    A_matrices
%     A1_0 = find_A_from_DH(a_vec(1),d_vec(1), alpha_vec(1), theta_vec(1));
%     A2_1 = find_A_from_DH(a_vec(2),d_vec(2), alpha_vec(2), theta_vec(2));
%     A3_2 = find_A_from_DH(a_vec(3),d_vec(3), alpha_vec(3), theta_vec(3));
%     A4_3 = find_A_from_DH(a_vec(4),d_vec(4), alpha_vec(4), theta_vec(4));
%     A5_4 = find_A_from_DH(a_vec(5),d_vec(5), alpha_vec(5), theta_vec(5));
%     A6_5 = find_A_from_DH(a_vec(6),d_vec(6), alpha_vec(6), theta_vec(6));
%     
%     A2_0 = A1_0 * A2_1;
%     A3_0 = A1_0 * A2_1 * A3_2;
%     A4_0 = A1_0 * A2_1 * A3_2 * A4_3;
%     A5_0 = A1_0 * A2_1 * A3_2 * A4_3 * A5_4;
%     A6_0 = A1_0 * A2_1 * A3_2 * A4_3 * A5_4 * A6_5;
    
%     A_matrices = [A1_0; A2_0 ; A3_0; A4_0; A5_0; A6_0];
 
%     Ares = zeros(4*6,4);
%     N = 1
%     temp =zeros(4,4);
    
    %     for i= 1:N
%         m = i-1
%         temp = find_A_from_DH(a_vec(i),d_vec(i), alpha_vec(i), theta_vec(i))
%         Ares(i+4-1:i+4-1,1:4) = temp
%     end

end