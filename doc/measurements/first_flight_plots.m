
load ('june_25/first_flight_attitude.mat', '-ASCII')

steps=find(first_flight_attitude(:,1))*0.02; %time in sec

figure(1)
%subplot(1,3,1)
%Plot magnet raw field x
plot(steps,first_flight_attitude(:,9)*1000,steps,first_flight_attitude(:,12));
hold on
LEGEND('magnet raw x','magnet comp x');
TITLE('magnet field data');

figure(2)
%subplot(1,3,2)
%Plot magnet raw field y
plot(steps,first_flight_attitude(:,10)*1000,steps,first_flight_attitude(:,13));
hold on
LEGEND('magnet raw y','magnet comp y');

subplot(1,3,3)
%Plot magnet raw field z
plot(steps,first_flight_attitude(:,11)*1000,steps,first_flight_attitude(:,14));
hold on
LEGEND('magnet raw z','magnet comp z');

figure(6)
%Plot magnitude of magnet vector
plot(steps,sqrt((first_flight_attitude(:,9)*1000).^2 + (first_flight_attitude(:,10)*1000).^2 + (first_flight_attitude(:,11)*1000).^2),steps,sqrt(first_flight_attitude(:,12).^2 + first_flight_attitude(:,13).^2 + first_flight_attitude(:,14).^2));
hold on
LEGEND('magnitude of magnet vector raw','magnitude of magnet vector comp.');

figure(2)
subplot(3,3,1)
%Plot acc x
plot(steps,first_flight_attitude(:,3));
hold on
LEGEND('acceleration x');

subplot(3,3,2)
%Plot acc y
plot(steps,first_flight_attitude(:,4));
hold on
LEGEND('acceleration y');

subplot(3,3,3)
%Plot acc z
plot(steps,first_flight_attitude(:,5));
hold on
LEGEND('acceleration z');

subplot(3,3,4)
%Plot gyro x
plot(steps,first_flight_attitude(:,6));
hold on
LEGEND('gyro x');

subplot(3,3,5)
%Plot gyro y
plot(steps,first_flight_attitude(:,7));
hold on
LEGEND('gyro y');

subplot(3,3,6)
%Plot gyro z
plot(steps,first_flight_attitude(:,8));
hold on
LEGEND('gyro z');

subplot(3,3,7)
%Plot attitude x (fusion)
plot(steps,first_flight_attitude(:,15)/1000);
hold on
LEGEND('attitude x (fusion)');

subplot(3,3,8)
%Plot attitude y (fusion)
plot(steps,first_flight_attitude(:,16)/1000);
hold on
LEGEND('attitude y (fusion)');

subplot(3,3,9)
%Plot attitude z (fusion)
plot(steps,first_flight_attitude(:,17)/1000);
hold on
LEGEND('attitude z (fusion)');

figure(3)

subplot(2,2,1)
%Plot attitude x (fusion)
plot(steps,first_flight_attitude(:,15)/1000);
hold on
LEGEND('attitude x (fusion)');

subplot(2,2,2)
%Plot attitude y (fusion)
plot(steps,first_flight_attitude(:,16)/1000);
hold on
LEGEND('attitude y (fusion)');

subplot(2,2,4)
%Plot angle sensor1
plot(steps,first_flight_attitude(:,1)/1000);
hold on
LEGEND('angle servo1(+y) (controller output)');

subplot(2,2,3)
%Plot angle sensor1
plot(steps,first_flight_attitude(:,2)/1000);
hold on
LEGEND('angle servo2(-y) (controller output)');

figure(4)
plot(steps,first_flight_attitude(:,1)/1000,steps,first_flight_attitude(:,16)/1000);
hold on
LEGEND('angle servo1(+y) (controller output)','attitude y (fusion)');

figure(5)
plot(steps,first_flight_attitude(:,2)/1000,steps,first_flight_attitude(:,15)/1000);
hold on
LEGEND('angle servo2(-y) (controller output)','attitude x (fusion)');
