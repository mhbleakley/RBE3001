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
magnification = 50;
%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end

    %Tuned with [100 0 0], [200 0 0], [0 -100 0] and [0 100 0]
    transMatrix = [0 1 0 100;
                   1 0 0 -50;
                   0 0 -1 0;
                   0 0 0 1;];
    

    pause(10);
    img = cam.getImage();
    checkerboard = checkerboard_Mask(img);
    redBall = redMask(img);
    greenBall = greenMask(img);
    yellowBall = yellowMask(img);
    orangeBall = orangeMask(img);
    imshow(checkerboard);
    figure, imshow(redBall);
    figure, imshow(greenBall);
    figure, imshow(yellowBall);
    figure, imshow(orangeBall);
%     imshow(img)
% Igray = rgb2gray(img);
% imshow(Igray);
% level = 0.67;
% Ithresh = im2bw(Igray,level);
% imshowpair(img, Ithresh, 'montage');

%     imHSV = rgb2hsv(img);
% 
%     % Get the saturation channel.
%     saturation = imHSV(:, :, 2);
% 
%     % Threshold the image
%     t = graythresh(saturation);
%     imBall = (saturation > t);
% 
%     figure; imshow(imBall, 'InitialMagnification', magnification);
%     title('Segmented Balls');

% Im=img;
% rmat=Im(:,:,1);
% gmat=Im(:,:,2);
% bmat=Im(:,:,3);
% subplot(2,2,1), imshow(rmat);
% title('Red Plane');% Igray = rgb2gray(img);
% imshow(Igray);
% level = 0.67;
% Ithresh = im2bw(Igray,level);
% imshowpair(img, Ithresh, 'montage');

% subplot(2,2,2), imshow(gmat);
% title('Green Plane');
% subplot(2,2,3), imshow(bmat);
% title('Blue Plane');
% subplot(2,2,4), imshow(I);
% title('Original Image');
% %%
% levelr = 0.63;
% levelg = 0.5;
% levelb = 0.4;
% i1=im2bw(rmat,levelr);
% i2=im2bw(gmat,levelg);
% i3=im2bw(bmat,levelb);
% Isum = (i1&i2&i3);
% % Plot the data
% subplot(2,2,1), imshow(i1);
% title('Red Plane');
% subplot(2,2,2), imshow(i2);
% title('Green Plane');
% subplot(2,2,3), imshow(i3);
% title('Blue Plane');
% subplot(2,2,4), imshow(Isum);
% title('Sum of all the planes');
% %% Complement Image and Fill in holes
% Icomp = imcomplement(Isum);
% Ifilled = imfill(Icomp,'holes');
% figure, imshow(Ifilled);
% %%
% se = strel('disk', 25);
% Iopenned = imopen(Ifilled,se);
% % figure,imshowpair(Iopenned, I);
% imshow(Iopenned);
% %% Extract features
% Iregion = regionprops(Iopenned, 'centroid');
% [labeled,numObjects] = bwlabel(Iopenned,4);
% stats = regionprops(labeled,'Eccentricity','Area','BoundingBox');
% areas = [stats.Area];
% eccentricities = [stats.Eccentricity];
% %% Use feature analysis to count skittles objects
% idxOfSkittles = find(eccentricities);
% statsDefects = stats(idxOfSkittles);
% figure, imshow(I);
% hold on;
% for idx = 1 : length(idxOfSkittles)
%         h = rectangle('Position',statsDefects(idx).BoundingBox,'LineWidth',2);
%         set(h,'EdgeColor',[.75 0 0]);
%         hold on;
% end
% if idx > 10
% title(['There are ', num2str(numObjects), ' objects in the image!']);
% end
% hold off;

% Igray = rgb2gray(img);
% imshow(Igray);
% level = 0.67;
% Ithresh = im2bw(Igray,level);
% imshowpair(img, Ithresh, 'montage');

catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

% self.cam = webcam('/dev/video0');