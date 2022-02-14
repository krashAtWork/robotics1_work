X = rotx(deg2rad(225));
Y = roty(deg2rad(40));
Z = rotz(deg2rad(90));
R=Z*Y*X;

o = [60; 80; 59];
A = [R o];
A = [A; 0 0 0 1]
Ainv = inv(A);

%check valid rotaiton matrix
dots = dot(R(:,1),R(:,2))+dot(R(:,2),R(:,3))+dot(R(:,3),R(:,1));
crosses = norm(cross(R(:,1),R(:,2)))+norm(cross(R(:,2),R(:,3)))+norm(cross(R(:,3),R(:,1)));
detR = det(R);
