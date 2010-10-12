

load('mavserial_markerlog_motor_on.txt','-ASCII');
load('mavserial_markerlog_motor_off.txt','-ASCII');
%data [MarkerNr, confidence,  pos_y, pos_z, pos_x, yaw, roll, nick]

data1 = mavserial_markerlog_motor_off;
data2 = mavserial_markerlog_motor_on;
steps1 = find(data1(:,1)~=1000000000000000000000000)*0.1;
steps2 = find(data2(:,1)~=1000000000000000000000000)*0.1;

figure(1)
plot(steps1, data1(:,5), steps1, data1(:,3), steps1, data1(:,4))
legend('x', 'y','z')
title('position in marker koordinate, motor off')

figure(2)
plot(steps2, data2(:,5), steps2, data2(:,3), steps2, data2(:,4))
legend('x', 'y','z')
title('position in marker koordinate, motor on')

figure(3)
plot(steps2, data2(:,6), steps2, data2(:,7), steps2, data2(:,8))
legend('yaw', 'roll','nick')
title('angle in marker koordinate, motor on')

figure(4)
plot(steps1, data1(:,6), steps1, data1(:,7), steps1, data1(:,8))
legend('yaw', 'roll','nick')
title('angle in marker koordinate, motor off')
