function n = planeNormal( Q, R, S )
QR = R - Q;
QS = S - Q;
n = cross(QR,QS);

% n(4) = n(1)*Q(1) + n(2)*Q(2) + n(3)*Q(3)
