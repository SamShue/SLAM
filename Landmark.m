classdef Landmark < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        landmarkObj;
        method;
        %correspondence;
    end
    
    methods
        function h = Landmark(method)
            h.method=method;
            switch(method)
                case 'RANSAC'
                    h.landmarkObj = RANSAC();
                otherwise
                    warning('Improper landmark recognition method.');
                    h.landmarkObj = RANSAC();
                    %h.correspondence = Correspondence(100000000000000000,100000000000000000);
            end
        end
        
        
        function [observed_LL] = getLandmark(h, laserdata, x)
            switch(h.method)
                case 'RANSAC'
                    observed_LL=h.landmarkObj.getLandmark(laserdata,x);
                otherwise
                    warning('Improper landmark recognition method.');   
            end
            
        end
        
    end
    
end

