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
red_place = [150, -50, 11];
orange_place = [150, 50, 11];
yellow_place = [75, -125, 11];
green_place = [75, 125, 11];
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
    pause(2.5);
    
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    
    if greenPoint ~= [-500; -500; -500]
        goToPoint = transpose(greenPoint);
        checkPoint = goToPoint(1, 1:3);
        robot.goto_ball(checkPoint,green_place);
        
    else
        disp("no green found")
    end
    
    % Re-check points
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    
    if redPoint ~= [-500; -500; -500]
        goToPoint = transpose(redPoint);
        checkPoint = goToPoint(1, 1:3);
        robot.goto_ball(checkPoint,red_place);
        
    else
        disp("no red found")
    end
    
    
    % Re-check points
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    
    if yellowPoint ~= [-500; -500; -500]
        goToPoint = transpose(yellowPoint);
        checkPoint = goToPoint(1, 1:3);
        robot.goto_ball(checkPoint,yellow_place);
        
    else
        disp("no yellow found")
    end
    
    
    
    % Re-check points
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    
    if orangePoint ~= [-500; -500; -500]
        goToPoint = transpose(orangePoint);
        checkPoint = goToPoint(1, 1:3);
        robot.goto_ball(checkPoint,orange_place);
        
    else
        disp("no orange found")
    end

catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

% self.cam = webcam('/dev/video0');