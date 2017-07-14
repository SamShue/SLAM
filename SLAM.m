classdef SLAM < handle
    %Objects of the slam class are responsible for running a single
    %instance of a SLAM algorithm. 
    %   This class will create an object of the appropriate SLAM agorithm
    %   as specified by the input string in the default constructor. The
    %   runSlam function can then be used to repeatedly call the predict
    %   and measurment functions of the appropriate SLAM algorithm. 
    %
    %   The LM variable is used to store the algorithm sepcific landmark
    %   object.
    %
    %   The slam variable is used to store the algorithm specific SLAM
    %   algorithm
    %
    %   The algorithName variable is used to store the name of the algorithm
    %   which is set when an object of this class is created. The
    %   algorithmName is also used to select each appropriate method to be
    %   called in the predict and measure functions
    %
    %   The laser variable stores laser scan data recieved from the kinect
    %   sensor
    %
    %   The odom variable stores odometry data recieved from the turtle bot.
    %
    %   The u vairable is used to store the control vector (change in linear
    %   displacement and rotation) of the turtlebot
    %
    %   oldOdomPose is used to keep track of the previous odom pose
    
    properties
        LM; 
        slam;
        algorithmName;
        laser;
        odom;
        
        u;
        oldOdomPose;
        
        chatpub
        msg
        map
        tftree
    end
    
    methods
        
        
                
        % Constructor - initialize state variables and covariances
        function h = SLAM(inputString)
            

            h.tftree=rostf;
            
            
            h.algorithmName = inputString;
            
            %initialize a subscriber node to kinect laser scan data
            h.laser = rossubscriber('/scan'); 
            
            %initialize a subscriber node to turtlebot odometry data
            h.odom = rossubscriber('/odom');
            
            %Switch case based on the algorithm name that creates objects
            %for the SLAM algorithm and landmark type 
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam = EKF_SLAM();
                    h.LM=Landmark('RANSAC');
                    h.u=[0;0;0];
               
                case 'EKF_SLAM_UC'
                    h.slam = EKF_SLAM_UC();
                    h.LM=Landmark('RANSAC');
                    h.u=[0;0;0];
               
                otherwise
                    
            end
            
            
            %Occupancy grid initalisation 
            h.map=robotics.BinaryOccupancyGrid(200,200,5);
            h.chatpub = rospublisher('/map','nav_msgs/OccupancyGrid');
            %  h.chatpub = rospublisher('/chatter','std_msgs/String');
            h.msg = rosmessage(h.chatpub);
            
            h.msg.Header.FrameId = 'map';
        end
        
            
               
                
        
        % Function - calls the predict function of the appropriate slam
        % algorithm 
        function predict(h,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.predict(u);
                case 'EKF_SLAM_UC' 
                    h.slam.predict(u);
                otherwise
            end
        end
        
        % Function - calls the measure function of the appropriate slam
        % algorithm 
        function measure(h,laserdata,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.measure(laserdata,u,h.LM);
                case 'EKF_SLAM_UC'
                    h.slam.measure(laserdata,u,h.LM);
            end   
        end
        
        % Function - calls the plot function of the appropriate slam
        % algorithm 
        function plot(h)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.plot(h.LM);
                case 'EKF_SLAM_UC'
                    h.slam.plot(h.LM);
            end
        end
        
        %Function - use to run one iteration of the appropriate SLAM
        %algorithm
        function runSlam(h)
            % Receive ROS Topics
            %======================================================================
            laserData  = receive(h.laser); %recieve a laser scan
            odomData = receive(h.odom);
           
            % End receive ROS topics
            %----------------------------------------------------------------------
            
            % Calculate Odometry
            %======================================================================
            % odometry data calculated here is the dead-reckoned position from
            % odometry readings in ROS.
            
            p = odomData.Pose.Pose;
            x_o = p.Position.X;
            y_o = p.Position.Y;
            
            quat = p.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = wrapTo360(rad2deg(angles(1)));
            
            % store current position in terms of x, y, theta (degrees) as ROS sees
            odomPose = [x_o,y_o,theta];
            % End calculate odometery
            %----------------------------------------------------------------------

            % Estimate Robot's pose
            %======================================================================
            % Initialize variables if first iteration
            if(isempty(h.oldOdomPose))
                h.oldOdomPose=odomPose;
            end
            % Get control vector (change in linear displacement and rotation)to
            % estimate current pose of the robot
            delta_D = sqrt((odomPose(1) - h.oldOdomPose(1))^2 + (odomPose(2) - h.oldOdomPose(2))^2);
            delta_Theta = rad2deg(angdiff(deg2rad(h.oldOdomPose(3)),deg2rad(odomPose(3))));
            h.u = [delta_D, delta_Theta];
            
            % Update position estimate
            h.slam.predict(h.u);
            
            % Record current odometry pose for next iteration
            h.oldOdomPose = odomPose;

            
            h.slam.measure(laserData, h.u,h.LM);
            
            cartes_data = readCartesian(laserData); %read cartesian co-ordinates
            
            rot = [cosd(h.slam.x(3)) -sind(h.slam.x(3)) h.slam.x(1)+12.5; sind(h.slam.x(3)) cosd(h.slam.x(3)) h.slam.x(2)+12.5; 0 0 1];
            world_frame_laser_scan = rot*[cartes_data,ones(length(cartes_data),1)]';
            

            
            
            
            
            %Plot scan data
            
            cartes_data = readCartesian(laserData); %read cartesian co-ordinates
            rot = [cosd(h.slam.x(3)) -sind(h.slam.x(3)) h.slam.x(1); sind(h.slam.x(3)) cosd(h.slam.x(3)) h.slam.x(2); 0 0 1];
            tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
            scatter(tmp(1,:),tmp(2,:),'magenta','.');
            axis([-3.5 3.5 -3.5 3.5]);
            
            
            %Plot slam data (landmarks, observed landmarks, root)
             figure(1);
            % h.slam.plot(h.LM);
            
            
            %Plot map
            world_frame_scan(1,:)=world_frame_laser_scan(1,:)+20;
            world_frame_scan(2,:)=world_frame_laser_scan(2,:)+20;
            
            figure(2);
            setOccupancy(h.map, world_frame_scan(1:2,:)',1);
            show(h.map);
         %   h.msg.Data='SAM';
           send(h.chatpub,h.msg);
           writeBinaryOccupancyGrid(h.msg,h.map);
           
            
           tfpub = rospublisher('/tf', 'tf2_msgs/TFMessage');   
           tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
           tfStampedMsg.ChildFrameId = 'odom';
           tfStampedMsg.Header.FrameId = 'map';
            if robotics.ros.internal.Global.isNodeActive
                tfStampedMsg.Header.Stamp = rostime('now');
            end
            tfStampedMsg.Transform.Translation.X = h.slam.x(2)+20;
            tfStampedMsg.Transform.Translation.Y = h.slam.x(1)+20;
            tfStampedMsg.Transform.Translation.Z = 0;
            tfStampedMsg.Transform.Rotation.W = quat.W;
            tfStampedMsg.Transform.Rotation.X = quat.X;
            tfStampedMsg.Transform.Rotation.Y = quat.Y;
            tfStampedMsg.Transform.Rotation.Z = quat.Z;



% tform = rosmessage('geometry_msgs/TransformStamped')
% tform.ChildFrameId = 'new_frame';
% tform.Header.FrameId = 'base_link';
% tform.Transform.Translation.X = 0.5;
% tform.Transform.Rotation.Z = 0.75;

             sendTransform(h.tftree, tfStampedMsg);
             send(h.chatpub,h.msg);
           
        end
    end
end
