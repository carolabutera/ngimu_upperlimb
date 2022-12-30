clear all
close all
clc

test=1; 

if test==1
   data=readmatrix("./validation/test1/ROTdata_2022-12-13-12-10-34.csv");
elseif test==2
   rot=readmatrix("./validation/test2/ROTdata_2022-12-13-12-25-46.csv");
elseif test==3
   rot=readmatrix("./validation/test3/ROTdata_2022-12-13-12-37-37.csv");
end
if test==1
   syn=readmatrix("./validation/test1/synchro_tiago_imu_test1.csv");
elseif test==2
   syn=readmatrix("./validation/test2/synchro_tiago_imu_test2.csv");
elseif test==3
   syn=readmatrix("./validation/test3/synchro_tiago_imu_test3.csv");
end





               
        
    
figure(1)
for i=11:19
    plot(syn(10000:15000,1),syn(10000:15000,i),'g')
    hold on

end

for i=29:31
    plot(syn(10000:15000,1),syn(10000:15000,i),'r')
    hold on

end


% 
% 
% figure(2)
% subplot(3,1,1)
% plot(data(:,1),data(:,20),'r')
% hold on
% plot(data(:,1),data(:,21),'g')
% hold on
% plot(data(:,1),data(:,22),'b')
% title("FA_x")
% xlabel("timestamp")
% legend("x","y","z")
% subplot(3,1,2)
% plot(data(:,1),data(:,23),'r')
% hold on
% plot(data(:,1),data(:,24),'g')
% hold on
% plot(data(:,1),data(:,25),'b')
% title("FA_y")
% xlabel("timestamp")
% legend("x","y","z")
% subplot(3,1,3)
% plot(data(:,1),data(:,26),'r')
% hold on
% plot(data(:,1),data(:,27),'g')
% hold on
% plot(data(:,1),data(:,28),'b')
% title("FA_z")
% xlabel("timestamp")
% legend("x","y","z")
% 
% 
% 
% 
% 

