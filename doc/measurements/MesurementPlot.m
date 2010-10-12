
load  stillstand_mes -ASCII
load  different_positions_mes -ASCII
load yaw_90_deg_left_mes -ASCII
load ('june_22/yaw_90deg_motors_off1.mat', '-ASCII')
load ('june_22/yaw_90deg_motors_off2.mat', '-ASCII')
load ('june_22/yaw_90deg_motors_off3.mat', '-ASCII')
load ('june_22/yaw_90deg_motors_on1.mat', '-ASCII')
load ('june_22/yaw_90deg_motors_on2.mat', '-ASCII')
load ('june_22/random_tilt_motors_on.mat', '-ASCII')
load ('june_23/different_mes_motors_on.mat', '-ASCII')

%accseleration
steps=find(stillstand_mes(:,1))*0.02;
plot(steps,stillstand_mes(:,1),steps,stillstand_mes(:,2),steps,stillstand_mes(:,3))

%rotation rates
steps=find(stillstand_mes(:,1))*0.02;
plot(steps,stillstand_mes(:,4),steps,stillstand_mes(:,5),steps,stillstand_mes(:,6))

%maget field
steps=find(stillstand_mes(:,1))*0.02;
plot(steps,stillstand_mes(:,7),steps,stillstand_mes(:,8),steps,stillstand_mes(:,9))
#############################################

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(different_positions_mes(:,1))*0.02;
plot(steps,different_positions_mes(:,1),steps,different_positions_mes(:,2),steps,different_positions_mes(:,3))

%rotation rates
steps=find(different_positions_mes(:,1))*0.02;
plot(steps,different_positions_mes(:,4),steps,different_positions_mes(:,5),steps,different_positions_mes(:,6))

%maget field
steps=find(different_positions_mes(:,1))*0.02;
plot(steps,different_positions_mes(:,7),steps,different_positions_mes(:,8),steps,different_positions_mes(:,9))
##############################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90_deg_left_mes(:,1))*0.02;
plot(steps,yaw_90_deg_left_mes(:,1),steps,yaw_90_deg_left_mes(:,2),steps,yaw_90_deg_left_mes(:,3))

%rotation rates
steps=find(yaw_90_deg_left_mes(:,1))*0.02;
plot(steps,yaw_90_deg_left_mes(:,4),steps,yaw_90_deg_left_mes(:,5),steps,yaw_90_deg_left_mes(:,6))

%maget field
steps=find(yaw_90_deg_left_mes(:,1))*0.02;
plot(steps,yaw_90_deg_left_mes(:,7),steps,yaw_90_deg_left_mes(:,8),steps,yaw_90_deg_left_mes(:,9))
################################################


################################################
%Jun2_22
################################################
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90deg_motors_off1(:,1))*0.02;
plot(steps,yaw_90deg_motors_off1(:,1),steps,yaw_90deg_motors_off1(:,2),steps,yaw_90deg_motors_off1(:,3))

%rotation rates
steps=find(yaw_90deg_motors_off1(:,1))*0.02;
plot(steps,yaw_90deg_motors_off1(:,4),steps,yaw_90deg_motors_off1(:,5),steps,yaw_90deg_motors_off1(:,6))

%maget field
steps=find(yaw_90deg_motors_off1(:,1))*0.02;
plot(steps,yaw_90deg_motors_off1(:,7),steps,yaw_90deg_motors_off1(:,8),steps,yaw_90deg_motors_off1(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90deg_motors_off2(:,1))*0.02;
plot(steps,yaw_90deg_motors_off2(:,1),steps,yaw_90deg_motors_off2(:,2),steps,yaw_90deg_motors_off2(:,3))

%rotation rates
steps=find(yaw_90deg_motors_off2(:,1));%*0.02;
plot(steps,yaw_90deg_motors_off2(:,4),steps,yaw_90deg_motors_off2(:,5),steps,yaw_90deg_motors_off2(:,6))

%maget field
steps=find(yaw_90deg_motors_off2(:,1))*0.02;
plot(steps,yaw_90deg_motors_off2(:,7),steps,yaw_90deg_motors_off2(:,8),steps,yaw_90deg_motors_off2(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90deg_motors_off3(:,1))*0.02;
plot(steps,yaw_90deg_motors_off3(:,1),steps,yaw_90deg_motors_off3(:,2),steps,yaw_90deg_motors_off3(:,3))

%rotation rates
steps=find(yaw_90deg_motors_off3(:,1));%*0.02;
plot(steps,yaw_90deg_motors_off3(:,4),steps,yaw_90deg_motors_off3(:,5),steps,yaw_90deg_motors_off3(:,6))

%maget field
steps=find(yaw_90deg_motors_off3(:,1))*0.02;
plot(steps,yaw_90deg_motors_off3(:,7),steps,yaw_90deg_motors_off3(:,8),steps,yaw_90deg_motors_off3(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90deg_motors_on1(:,1))*0.02;
plot(steps,yaw_90deg_motors_on1(:,1),steps,yaw_90deg_motors_on1(:,2),steps,yaw_90deg_motors_on1(:,3))

%rotation rates
steps=find(yaw_90deg_motors_on1(:,1));%*0.02;
plot(steps,yaw_90deg_motors_on1(:,4),steps,yaw_90deg_motors_on1(:,5),steps,yaw_90deg_motors_on1(:,6))

%maget field
steps=find(yaw_90deg_motors_on1(:,1))*0.02;
plot(steps,yaw_90deg_motors_on1(:,7),steps,yaw_90deg_motors_on1(:,8),steps,yaw_90deg_motors_on1(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(yaw_90deg_motors_on2(:,1))*0.02;
plot(steps,yaw_90deg_motors_on2(:,1),steps,yaw_90deg_motors_on2(:,2),steps,yaw_90deg_motors_on2(:,3))

%rotation rates
steps=find(yaw_90deg_motors_on2(:,1));%*0.02;
plot(steps,yaw_90deg_motors_on2(:,4),steps,yaw_90deg_motors_on2(:,5),steps,yaw_90deg_motors_on2(:,6))

%maget field
steps=find(yaw_90deg_motors_on2(:,1))*0.02;
plot(steps,yaw_90deg_motors_on2(:,7),steps,yaw_90deg_motors_on2(:,8),steps,yaw_90deg_motors_on2(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(random_tilt_motors_on(:,1))*0.02;
plot(steps,random_tilt_motors_on(:,1),steps,random_tilt_motors_on(:,2),steps,random_tilt_motors_on(:,3))

%rotation rates
steps=find(random_tilt_motors_on(:,1));%*0.02;
plot(steps,random_tilt_motors_on(:,4),steps,random_tilt_motors_on(:,5),steps,random_tilt_motors_on(:,6))

%maget field
steps=find(random_tilt_motors_on(:,1))*0.02;
plot(steps,random_tilt_motors_on(:,7),steps,random_tilt_motors_on(:,8),steps,random_tilt_motors_on(:,9))
################################################


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accseleration
steps=find(different_mes_motors_on(:,1))*0.02;
plot(steps,different_mes_motors_on(:,1),steps,different_mes_motors_on(:,2),steps,different_mes_motors_on(:,3))

%rotation rates
steps=find(different_mes_motors_on(:,1));%*0.02;
plot(steps,different_mes_motors_on(:,4),steps,different_mes_motors_on(:,5),steps,different_mes_motors_on(:,6))

%maget field
steps=find(different_mes_motors_on(:,1))*0.02;
plot(steps,different_mes_motors_on(:,7),steps,different_mes_motors_on(:,8),steps,different_mes_motors_on(:,9))
################################################








%accseleration
mean(stillstand_mes(:,1));
mean(stillstand_mes(:,2));
mean(stillstand_mes(:,3));

steps=find(stillstand_mes(:,1))*0.02;
plot(steps,stillstand_mean(:,1),steps,stillstand_mean(:,2),steps,stillstand_mean(:,3))

%gyro bias values
mean(stillstand_mes(:,4));
mean(stillstand_mes(:,5));
mean(stillstand_mes(:,6));
