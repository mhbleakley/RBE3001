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
    pause(10);
    
    
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase);
    none_present = 0;
    
    if redPoint(1,1) == -500 && orangePoint(1,1) == -500 && yellowPoint(1,1) == -500 && greenPoint(1,1) == -500 && bluePoint(1,1) == -500
        none_present = 1;
    end
    
    while(none_present == 0)
        
        greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
        if greenPoint ~= no_ball_found
            goToPoint = transpose(greenPoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,green_angle);
            
        else
            disp("no green found")
        end
        
        redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
        if redPoint ~= no_ball_found
            goToPoint = transpose(redPoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,red_angle);
            
        else
            disp("no red found")
        end
        
        yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
        if yellowPoint ~= no_ball_found
            goToPoint = transpose(yellowPoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,yellow_angle);
            
        else
            disp("no yellow found")
        end
        
        orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
        if orangePoint ~= no_ball_found
            goToPoint = transpose(orangePoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,orange_angle);
            
        else
            disp("no orange found")
        end

        bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase);
        if bluePoint ~= no_ball_found
            goToPoint = transpose(bluePoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,blue_angle);
            
        else
            disp("no blue found")
        end
        
        redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
        orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
        yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
        greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
        bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase);
        
        if redPoint(1,1) == -500 && orangePoint(1,1) == -500 && yellowPoint(1,1) == -500 && greenPoint(1,1) == -500 && bluePoint(1,1) == -500
            none_present = 1;
        end
        
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