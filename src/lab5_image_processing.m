%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
traj = Traj_Planner();

cam = Camera();
cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11];
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];
magnification = 50;
Intrinsics = cam.cam_imajl;
Extrinsics = cam.cam_pose;

    transMatrix = [0 1 0 100;
                   1 0 0 -50;
                   0 0 -1 0;
                   0 0 0 1;];
               
    %Inverse of transMatrix           
    checkerToBase = [0 1 0 50;
                     1 0 0 -100;
                     0 0 -1 0;
                     0 0 0 1];

%% Main Loop
try
    
    zeroPoint = [100, 0, 195];
    robot.interpolate_jp(robot.ik3001(zeroPoint), 2000);
    
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    disp("done calibrating")
  
    pause(5);
    img = cam.getImage();
    checkerboard = checkerboard_Mask(img);

    % Finding Centroid Green Ball
    greenBall = greenMask(img);
    exgreenFill = imfill(greenBall,'holes');
    greenFill = xor(bwareaopen(exgreenFill,25),  bwareaopen(exgreenFill,1000));
    green = regionprops(greenFill);
    greenCentroid = green.Centroid;
    greenChecker = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), greenCentroid);
    greenPoint(1:2,1) = greenChecker;
    greenPoint(3,1) = 0;
    greenPoint(4,1) = 1;
    baseGreenPoint = checkerToBase*greenPoint
    newGreenPoint(1,1) = (((baseGreenPoint(1,1))^3)*0.0000153556631) + (((baseGreenPoint(1,1))^2)*-0.00206258926) + (((baseGreenPoint(1,1))*0.972513954)) + 13.6730092; 
    newGreenPoint(2,1) = (((baseGreenPoint(2,1))^3)*-0.00000158817254) + (((baseGreenPoint(2,1))^2)*0.00111751456) + (((baseGreenPoint(2,1)))*0.951520909) - 12.9545212; 
    newGreenPoint(3,1) = 15;
    
    goToPoint = transpose(newGreenPoint);
    checkPoint = goToPoint(1, 1:3);
    tic
    while toc < 5.2
        if ~(robot.finished_movement(zeroPoint, checkPoint)) && (toc <= 3)
        x = traj.linear_traj(100, checkPoint(1), 0, 3, toc);
        y = traj.linear_traj(0, checkPoint(2), 0, 3, toc);
        robot.interpolate_jp(robot.ik3001([x, y, 195]),0);
        elseif ~(robot.finished_movement(zeroPoint, checkPoint)) && toc > 3 && toc <= 5
        z = traj.linear_traj(195, checkPoint(3), 3, 5, toc);
        robot.interpolate_jp(robot.ik3001([x, y, z]),0);
        end
    end
    robot.interpolate_jp(robot.ik3001(zeroPoint), 2000);
    pause(5);

    % Finding Centroid Red Ball
%     redBall = redMask(img);
%     redFill = imfill(redBall,'holes');
%     red = regionprops(redFill);
%     redCentroid = red.Centroid;
%     redChecker = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), redCentroid);
%     redPoint(1:2,1) = redChecker;
%     redPoint(3,1) = 0;
%     redPoint(4,1) = 1;
%     redPoint = checkerToBase*redPoint


%     % Finding Centroid Yellow Ball
%     yellowBall = yellowMask(img);
%     yellowFill = imfill(yellowBall,'holes');
%     yellow = regionprops(yellowFill);
%     yellowCentroid = yellow.Centroid;
%     yellowChecker = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), yellowCentroid);
%     yellowPoint(1:2,1) = yellowChecker;
%     yellowPoint(3,1) = 0;
%     yellowPoint(4,1) = 1;
%     yellowPoint = checkerToBase*yellowPoint
%     
%     figure, imshow(redFill)
%     figure, imshow(yellowFill)
%     figure, imshow(greenFill)


    % Finding Centroid Orange Ball
%     orangeBall = orangeMask(img);
%     orangeFill = imfill(orangeBall,'holes');
%     orange = regionprops(orangeFill);
%     orangeCentroid = orange.Centroid;
%     orangeChecker = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), orangeCentroid);
%     orangePoint(1:2,1) = orangeChecker;
%     orangePoint(3,1) = 0;
%     orangePoint(4,1) = 1;
%     orangePoint = checkerToBase*orangePoint   

    


catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

% self.cam = webcam('/dev/video0');