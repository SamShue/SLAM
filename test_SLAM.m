% 
 clc;      
 clear all;
 %clear command window
 close all;                          %close all figures
 rosshutdown                         %close current ros incase it is already initalized
% 
% % Robot network variables
% ipaddress = 'http://192.168.1.16:11311';         %define ipadress of turtlebot
% %setenv('ROS_MASTER_URI', ipaddress);
% %rosinit(ipaddress,'NodeHost','192.168.1.133')                  %initate ros using turtlebot IP
 rosinit('192.168.1.15');



s=GRAPH_SLAM('/odom','/scan','RANSAC');

while(1)
s.run();
end 









% t=timer;
% t.StartFcn = @initTimer;
% t.TimerFcn = @timerCallback;
% t.Period   = 0.2;
% t.TasksToExecute = inf;
% t.ExecutionMode  = 'fixedRate';
% start(t);
% 
% 
% function initTimer(src, event)
%        disp('initialised')
%        src.UserData=test_slam_class();
% end
% function timerCallback(src, event)
%         src.UserData.runSlam();
% end