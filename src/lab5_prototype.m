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
red_angle = [-90 90 -60];
orange_angle = [90 90 -60];
green_angle = [-60 90 -60];
yellow_angle = [70 90 -60];
blue_angle = [70 90 -60];

% Camera Parameters
Intrinsics = cam.cam_imajl;
Extrinsics = cam.cam_pose;
no_ball_found = [-500; -500; -500]; % Equal to the output of cam.findBall() when no ball of the chosen color is found

    transMatrix = [0 1 0 100;
                   1 0 0 -50;
                   0 0 -1 0;
                   0 0 0 1;];
               
    %Inverse of transMatrix, goes from checkerboard frame to the base frame           
    checkerToBase = [0 1 0 50;
                     1 0 0 -100;
                     0 0 -1 0;
                     0 0 0 1];

%% Main Loop
try
    % Send the robot to the zero position and opens the gripper
    zeroPoint = [100, 0, 195];
    robot.interpolate_jp(robot.ik3001(zeroPoint), 2000);
    robot.openGripper();
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    disp("done calibrating")
    pause(10);
    
    % Look for balls of each color
    redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
    orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
    yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
    greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
    bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase);
    none_present = 0;
    
    if redPoint(1,1) == -500 && orangePoint(1,1) == -500 && yellowPoint(1,1) == -500 && greenPoint(1,1) == -500 && bluePoint(1,1) == -500
        none_present = 1;
    end
    
    while(none_present == 0) % While there is at least one ball present on the checkerboard
        
        greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase); % Look for green balls
        if greenPoint ~= no_ball_found
            goToPoint = transpose(greenPoint); % Transposes the detected centroid for green
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,green_angle);  % Goes to the green ball closest to the camera origin, picks it up, and drops it off
            
        else
            disp("no green found")
        end
        
        redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase); % Look for red balls
        if redPoint ~= no_ball_found
            goToPoint = transpose(redPoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,red_angle);
            
        else
            disp("no red found")
        end
        
        yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase); % Look for yellow balls
        if yellowPoint ~= no_ball_found
            goToPoint = transpose(yellowPoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,yellow_angle);
            
        else
            disp("no yellow found")
        end
        
        orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase); % Look for orange balls
        if orangePoint ~= no_ball_found
            goToPoint = transpose(orangePoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,orange_angle);
            
        else
            disp("no orange found")
        end

        bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase); % Look for blue balls
        if bluePoint ~= no_ball_found
            goToPoint = transpose(bluePoint);
            checkPoint = goToPoint(1, 1:3);
            robot.goto_ball(checkPoint,blue_angle);
            
        else
            disp("no blue found")
        end
        
        % Check again to see if any balls remain
        redPoint = cam.findBall("red",Intrinsics,Extrinsics,checkerToBase);
        orangePoint = cam.findBall("orange",Intrinsics,Extrinsics,checkerToBase);
        yellowPoint = cam.findBall("yellow",Intrinsics,Extrinsics,checkerToBase);
        greenPoint = cam.findBall("green",Intrinsics,Extrinsics,checkerToBase);
        bluePoint = cam.findBall("blue",Intrinsics,Extrinsics,checkerToBase);
        
        % If no balls are left, end the program
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