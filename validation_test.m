clear all
close all
clc

test=6; 
data=import_data(test);


timestamp=data(:,1);
POE_imu=data(:,2);
POE_tiago=data(:,3);
AOE_imu=data(:,4);
AOE_tiago=data(:,5);
HR_imu=data(:,6);
HR_tiago=data(:,7);
FE_imu=data(:,8);
FE_tiago=data(:,9);
PS_imu=data(:,10);
PS_tiago=data(:,11);


%RMSE
for i=1:size(data,1)
    POE_diff(i)=(data(i,2)-data(i,3))^2;
    AOE_diff(i)=(data(i,4)-data(i,5))^2;
    HR_diff(i)=(data(i,6)-data(i,7)+pi/2)^2;
    FE_diff(i)=(data(i,8)-data(i,9))^2;    
    PS_diff(i)=(data(i,10)-data(i,11))^2;
end

POE_sum=sum(POE_diff(:));
AOE_sum=sum(AOE_diff(:));
HR_sum=sum(HR_diff(:));
FE_sum=sum(FE_diff(:));
PS_sum=sum(PS_diff(:));

POE_rmse=(1/size(data,1))*sqrt(POE_sum);
AOE_rmse=(1/size(data,1))*sqrt(AOE_sum);
HR_rmse=(1/size(data,1))*sqrt(HR_sum);
FE_rmse=(1/size(data,1))*sqrt(FE_sum);
PS_rmse=(1/size(data,1))*sqrt(PS_sum);

 %absolute errors 

POE_err=abs(POE_imu-POE_tiago);
AOE_err=abs(AOE_imu-AOE_tiago);
HR_err=abs(HR_imu-HR_tiago+pi/2);
FE_err=abs(FE_imu-FE_tiago);
PS_err=abs(PS_imu-PS_tiago);

%plot of absolute errors
figure(1)
subplot(5,1,1)
plot(timestamp, POE_err*180/pi,'r')
title("POE absolute error")
xlabel("timestamp")
ylabel("absolute error (°)")

subplot(5,1,2)
plot(timestamp, AOE_err*180/pi,'r')
title("AOE absolute error")
xlabel("timestamp")
ylabel("absolute error (°)")

subplot(5,1,3)
plot(timestamp, (HR_err)*180/pi,'r')
title("HR absolute error")
xlabel("timestamp")
ylabel("absolute error (°)")

subplot(5,1,4)
plot(timestamp, FE_err*180/pi,'r')
title("FE absolute error")
xlabel("timestamp")
ylabel("absolute error (°)")

subplot(5,1,5)
plot(timestamp, PS_err*180/pi,'r')
title("PS absolute error")
xlabel("timestamp")
ylabel("absolute error (°)")


%plot of angles
figure(2)
subplot(5,1,1)
plot(timestamp, POE_imu*180/pi,'r')
hold on
plot(timestamp, POE_tiago*180/pi,'g')
title("POE")
xlabel("timestamp")
ylabel("angle(°)")
legend("NGIMU","TIAGO")

subplot(5,1,2)
plot(timestamp, AOE_imu*180/pi,'r')
hold on
plot(timestamp, AOE_tiago*180/pi,'g')
title("AOE")
xlabel("timestamp")
ylabel("angle(°)")
legend("NGIMU","TIAGO")

subplot(5,1,3)
plot(timestamp, HR_imu*180/pi,'r')
hold on
plot(timestamp, (HR_tiago-pi/2)*180/pi,'g')
title("HR")
xlabel("timestamp")
ylabel("angle(°)")
legend("NGIMU","TIAGO")

subplot(5,1,4)
plot(timestamp, FE_imu*180/pi,'r')
hold on
plot(timestamp, FE_tiago*180/pi,'g')
title("FE")
xlabel("timestamp")
ylabel("angle(°)")
legend("NGIMU","TIAGO")

subplot(5,1,5)
plot(timestamp,PS_imu*180/pi,'r')
hold on
plot(timestamp, PS_tiago*180/pi,'g')
title("PS")
xlabel("timestamp")
ylabel("angle(°)")
legend("NGIMU","TIAGO")
