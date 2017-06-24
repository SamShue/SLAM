classdef SLAM < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        LM; 
        slam;
        algorithmName;
        laser;
        odom;
        
        u;
        oldOdomPose;
        
        
    end
    
    methods
        
        % Constructor - initialize state variables and covariances
        function h = SLAM(inputString)
            h.algorithmName = inputString;
            h.laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
            h.odom = rossubscriber('/odom');
            
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
        end
        
        function predict(h,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.predict(u);
                case 'EKF_SLAM_UC' 
                    h.slam.predict(u);
                otherwise
            end
        end
        
        function measure(h,laserdata,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.measure(laserdata,u,h.LM);
                case 'EKF_SLAM_UC'
                    h.slam.measure(laserdata,u,h.LM);
            end   
        end
        
        function plot(h)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.plot(h.LM);
                case 'EKF_SLAM_UC'
                    h.slam.plot(h.LM);
            end
        end
        
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
            
            %setOccupancy(map, world_frame_laser_scan(1:2,:)',1);
            %show(map);
            
            
            
            
            %Plot scan data
            
            cartes_data = readCartesian(laserData); %read cartesian co-ordinates
            rot = [cosd(h.slam.x(3)) -sind(h.slam.x(3)) h.slam.x(1); sind(h.slam.x(3)) cosd(h.slam.x(3)) h.slam.x(2); 0 0 1];
            tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
            scatter(tmp(1,:),tmp(2,:),'magenta','.');
            axis([-3.5 3.5 -3.5 3.5]);
            
            
            %Plot slam data (landmarks, observed landmarks, root)
            h.slam.plot(h.LM);
            
            
            
            
        end
    end
end
