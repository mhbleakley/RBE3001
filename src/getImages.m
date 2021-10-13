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
% red_place = [150, -50, 25];
% orange_place = [150, 50, 25];
% yellow_place = [75, -150, 25];
% green_place = [75, 150, 25];
red_angle = [-90 90 -60];
orange_angle = [90 90 -60];
green_angle = [-60 90 -60];
yellow_angle = [70 90 -60];
blue_angle = [70 90 -60];

Intrinsics = cam.cam_imajl;
Extrinsics = cam.cam_pose;
no_ball_found = [-500; -500; -500];

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
    robot.openGripper();
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    
    disp("done calibrating")
    pause(5);
    
    img = cam.getImage();
    figure, imshow(img);
    img = yellowMask(img);
    figure, imshow(img);
    exFill = imfill(img,'holes');
    figure, imshow(exFill);
    fill = xor(bwareaopen(exFill,25),  bwareaopen(exFill,1000));
    figure, imshow(fill);
    
    

catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

% self.cam = webcam('/dev/video0');