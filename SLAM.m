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
        slam;
        algorithmName;
        occupancyGrid;
    end
    
    methods       
        % Constructor - initialize state variables and covariances
        function h = SLAM(inputString)
            h.algorithmName = inputString;
            
            %Switch case based on the algorithm name that creates objects
            %for the SLAM algorithm and landmark type 
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam = EKF_SLAM('/odom','/scan','RANSAC');
                case 'EKF_SLAM_UC'
                    h.slam = EKF_SLAM_UC('/odom','/scan','RANSAC');
                otherwise
                    
            end
            
            h.occupancyGrid = OccupancyGrid();
        end
        
        function runSlam(h)
           h.slam.run();
           % Plot slam data (landmarks, observed landmarks, root)
           h.slam.plot();
           h.slam.LM.plot();
           axis([-5 5 -5 5]);
        end
    end
end
