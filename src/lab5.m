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

cam = Camera();
cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11];
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
%     cam.cam_pose = cam.getCameraPose();
%     disp(cam.cam_pose);
    
%     img = snapshot(cam.cam);
%     imshow(img);


%   Point of interest, X-Y pixels:  399.0000  310.0000
%     img = cam.cam.snapshot();
%     [undistorted_img,pseudo_intrinsics] = undistortFisheyeImage(img, cam.params.Intrinsics, 'OutputView', 'full');
%     imshow(undistorted_img);
%     POI1 = ginput(1);

img = cam.getImage();
imshow(img)
POI1 = ginput(1)
POI2 = ginput(1)
POI3 = ginput(1)
POI4 = ginput(1)
    
%     POI1 = [123, 305];
%     POI2 = [243, 270];
%     POI3 = [399, 310];
%     POI4 = [360, 108];
    
    Intrinsics = cam.cam_imajl;
    Extrinsics = cam.cam_pose;
    
%     Vector in checkerboard frame
    P1 = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), POI1)
    P2 = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), POI2)
    P3 = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), POI3)
    P4 = pointsToWorld(Intrinsics, Extrinsics(1:3, 1:3), Extrinsics (1:3, 4), POI4)

    %Tuned with [100 0 0], [200 0 0], [0 -100 0] and [0 100 0]
    transMatrix = [0 1 0 100;
                   1 0 0 -50;
                   0 0 -1 0;
                   0 0 0 1;];
    
    %Inverse of transMatrix           
    checkerToBase = [0 1 0 50;
                     1 0 0 -100;
                     0 0 -1 0;
                     0 0 0 1];
        
    checkPoint1 = transMatrix * [75; 0; 0; 1]
    checkPoint2 = transMatrix * [150; 0; 0; 1]
    checkPoint3 = transMatrix * [-25; -100; 0; 1]
    checkPoint4 = transMatrix * [-25; 100; 0; 1]


catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

% self.cam = webcam('/dev/video0');