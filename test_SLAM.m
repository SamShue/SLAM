% 
% clc;                                %clear command window
% clear all;
% close all;                          %close all figures
% rosshutdown                         %close current ros incase it is already initalized
% 
% % Robot network variables
% ipaddress = 'http://192.168.1.16:11311';         %define ipadress of turtlebot
% %setenv('ROS_MASTER_URI', ipaddress);
% %rosinit(ipaddress,'NodeHost','192.168.1.133')                  %initate ros using turtlebot IP
% rosinit('192.168.1.16');
clc;

u=[1;90;...
   1;-90;...
  -1;90;...
  -1;-90];

c={[],[1],[1,2],[2]};
z={[],[10,10],[10,10,20,20],[20,20]};

s=GRAPH_SLAM();


s.runSlam(u,z,c);










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