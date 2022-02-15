%forward dynamics: given joint angles, joint velocities and joint torques, 
%compute qddot, joint accelerations

function [qddot] = fwd_dynamics (a_vec,q_vec,qdot_vec,tau_vec)
%  a_vec
[H,c_vec] = compute_H_and_cvec (q_vec, qdot_vec,a_vec);
qddot = inv(H)*(tau_vec-c_vec);

end
