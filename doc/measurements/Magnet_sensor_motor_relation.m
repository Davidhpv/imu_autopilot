
load ('june_23/stillstand_motors_off.mat', '-ASCII')
load ('june_23/stillstand_motors_on1.mat', '-ASCII')
load ('june_23/stillstand_motors_on2.mat', '-ASCII')
load ('june_23/stillstand_without_motors.mat', '-ASCII')


steps1=find(stillstand_motors_off(:,1))*0.02;
plot(steps1,stillstand_motors_off(:,7),steps1,stillstand_motors_off(:,8),steps1,stillstand_motors_off(:,9));
%#############################################


%maget field
steps2=find(stillstand_motors_on2(:,1))*0.02;
plot(steps2,stillstand_motors_on2(:,7),steps2,stillstand_motors_on2(:,8),steps2,stillstand_motors_on2(:,9));
%#############################################


%maget field
steps3=find(stillstand_without_motors(:,1))*0.02;
plot(steps3,stillstand_without_motors(:,7),steps3,stillstand_without_motors(:,8),steps3,stillstand_without_motors(:,9));
%#############################################

%maget field
steps4=find(stillstand_motors_on1(:,1))*0.02;
plot(steps4,stillstand_motors_on1(:,7),steps4,stillstand_motors_on1(:,8),steps4,stillstand_motors_on1(:,9));
%#############################################

figure(1)
subplot(2,3,1)
%Plot x-magnet field
plot(steps1,stillstand_motors_off(:,7),steps2,stillstand_motors_on2(:,7),steps3,stillstand_without_motors(:,7));
LEGEND('motor off','motor on','whitout motor');
TITLE('x-magnet field');

subplot(2,3,2)
%Plot y-magnet field
plot(steps1,stillstand_motors_off(:,8),steps2,stillstand_motors_on2(:,8),steps3,stillstand_without_motors(:,8));
LEGEND('motor off','motor on','whitout motor');
AXIS([0 14 -10 130]);
TITLE('y-magnet field');

subplot(2,3,3)
%Plot z-magnet field
plot(steps1,stillstand_motors_off(:,9),steps2,stillstand_motors_on2(:,9),steps3,stillstand_without_motors(:,9));
LEGEND('motor off','motor on','whitout motor');
AXIS([0 14 200 420]);
TITLE('z-magnet field');

subplot(2,3,4)
%Plot roll angle(x)
plot(steps1,stillstand_motors_off(:,10),steps2,stillstand_motors_on2(:,10),steps3,stillstand_without_motors(:,10));
LEGEND('motor off','motor on','whitout motor');
TITLE('roll angle (x) from sensor fusion');

subplot(2,3,5)
%Plot nick angle(y)
plot(steps1,stillstand_motors_off(:,11),steps2,stillstand_motors_on2(:,11),steps3,stillstand_without_motors(:,11));
LEGEND('motor off','motor on','whitout motor');
TITLE('nick angle(y) from sensor fusion');

subplot(2,3,6)
%Plot yaw angle(z)
plot(steps1,stillstand_motors_off(:,12),steps2,stillstand_motors_on2(:,12),steps3,stillstand_without_motors(:,12));
LEGEND('motor off','motor on','whitout motor');
TITLE('yaw angle(z) from sensor fusion');

%motor starup data
figure(2)
subplot(2,3,1)
%Plot x-magnet field
plot(steps1,stillstand_motors_off(:,7),steps4,stillstand_motors_on1(:,7),steps3,stillstand_without_motors(:,7));
LEGEND('motor off','motor on','whitout motor');
TITLE('x-magnet field');

subplot(2,3,2)
%Plot y-magnet field
plot(steps1,stillstand_motors_off(:,8),steps4,stillstand_motors_on1(:,8),steps3,stillstand_without_motors(:,8));
LEGEND('motor off','motor on','whitout motor');
AXIS([0 14 -10 130]);
TITLE('y-magnet field');

subplot(2,3,3)
%Plot z-magnet field
plot(steps1,stillstand_motors_off(:,9),steps4,stillstand_motors_on1(:,9),steps3,stillstand_without_motors(:,9));
LEGEND('motor off','motor on','whitout motor');
AXIS([0 14 200 420]);
TITLE('z-magnet field');

subplot(2,3,4)
%Plot roll angle(x)
plot(steps1,stillstand_motors_off(:,10),steps4,stillstand_motors_on1(:,10),steps3,stillstand_without_motors(:,10));
LEGEND('motor off','motor on','whitout motor');
TITLE('roll angle (x) from sensor fusion');

subplot(2,3,5)
%Plot nick angle(y)
plot(steps1,stillstand_motors_off(:,11),steps4,stillstand_motors_on1(:,11),steps3,stillstand_without_motors(:,11));
LEGEND('motor off','motor on','whitout motor');
TITLE('nick angle(y) from sensor fusion');

subplot(2,3,6)
%Plot yaw angle(z)
plot(steps1,stillstand_motors_off(:,12),steps4,stillstand_motors_on1(:,12),steps3,stillstand_without_motors(:,12));
LEGEND('motor off','motor on','whitout motor');
TITLE('yaw angle(z) from sensor fusion');
