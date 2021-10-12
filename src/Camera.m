classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;  
        DEBUG_BALLDETECTION = false;
        
        % Properties
        params;
        cam;
        cam_pose;
        cam_imajl;
        testIn
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(2); % Get camera object
%             self.params = self.calibrate(); % Run Calibration Function
            load("cam_workspace", "cameraParams");
            self.params = cameraParams;
            self.testIn = cameraParams.Intrinsics;
            [self.cam_imajl, self.cam_pose] = self.getCameraPose();
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALOBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                Intrinsic_cal; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("This is REEE Frogg:")
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camerea calibration file found. Plese run camera calibration");
            end          
        end
        
        %Identifies the centroid of an object of a specified color, the
        %in/extrinsic parameters and the checker to base frame matrix
        function obj_point = findBall(self,color,Intrinsics,Extrinsics,checkerToBase)
            img = self.getImage();
            
            % Apply mask based upon chosen color
            if color == "green"
                img = greenMask(img);
            end
            if color == "red"
                img = redMask(img);
            end
            if color == "yellow"
                img = yellowMask(img);
            end
            if color == "orange"
                img = orangeMask(img);
            end
            
            try
                % Image Processing
                exFill = imfill(img,'holes');
                fill = xor(bwareaopen(exFill,25),  bwareaopen(exFill,1000));
                ball = regionprops(fill);
                centroid = ball.Centroid;
                checker = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), centroid);
                Point(1:2,1) = checker;
                Point(3,1) = 0;
                Point(4,1) = 1;
                basePoint = checkerToBase*Point;
                
                % Correction math
                xPoint = basePoint(1,1);
                yPoint = basePoint(2,1);
                
                xAngle = atan2((200-xPoint), 260);
                yAngle = atan2(yPoint, 260);
               
 
                if yPoint > 0
                     if xPoint < 50
                        xCorrection = (11 * (tan(xAngle))) - 25;
                        yCorrection = (11 * (tan(yAngle))) - 4;
                     else
                        xCorrection = (11 * (tan(xAngle))) - 15;
                        yCorrection = (11 * (tan(yAngle))) + 9;
                     end
                else
                    if xPoint < 50 
                        yCorrection = (11 * (tan(yAngle))) + 9;
                        xCorrection = (11 * (tan(xAngle)));
                    else
                        xCorrection = (11 * (tan(xAngle)));
                        yCorrection = (11 * (tan(yAngle))) + 20;
                    end
                end
                
                newPoint(1,1) = basePoint(1,1) - xCorrection;
                newPoint(2,1) = basePoint(2,1) - yCorrection;
                newPoint(3,1) = 10;
                obj_point = newPoint;
                
%                 if (newPoint(1,1) >= -25 &&  newPoint(1,1) <= 150 && newPoint(2,1) >= -125 && newPoint(2,1) <= 125)
%                     obj_point = newPoint;
%                 else
%                     %Centroid out of checkerboard
%                     obj_point = [-500; -500; -500];
%                 end
%                 
            catch exception
                % No centroid found
                obj_point = [-500; -500; -500];
%                 disp(getReport(exception))
            end
            
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.testIn, 'OutputView', 'full');
        end

        
        function [newIs, pose] = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
%             % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            self.params.Intrinsics = newIs;
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params.Intrinsics);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                
                figure(10)
                newImgPoints = [];
                c = 1;
                for i = 0:25:175
                    for j = 0:25:125
                        newImgPoints(c,1) = i;
                        newImgPoints(c,2) = j;
                        newImgPoints(c,3) = 0;
                        c = c + 1;
                    end
                end
                
                newGridPoints = worldToImage(newIs, R, t, newImgPoints);

                axesPoints = worldToImage(newIs, R, t, [0 0 0; 0 50 0; 50 0 0]);

                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                hold on 
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                viscircles(newGridPoints, ones(length(newGridPoints),1)*5);
                hold off
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end     
        end
    end
end