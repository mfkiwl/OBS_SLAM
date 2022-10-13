function C = quattorot(quat)

q1 = quat(1);
q2 = quat(2);
q3 = quat(3);
q4 = quat(4);

C = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2+q3*q4),2*(q1*q3-q2*q4);
    2*(q1*q2-q3*q4),-q1^2+q2^2-q3^2+q4^2, 2*(q2*q3+q1*q4);
    2*(q1*q3+q2*q4),2*(q2*q3-q1*q4),-q1^2-q2^2+q3^2+q4^2];

end