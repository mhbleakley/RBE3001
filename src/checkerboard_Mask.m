function [BW,maskedRGBImage] = checkerboard_Mask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 05-Oct-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.047;
channel1Max = 0.696;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 0.261;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.394;
channel3Max = 0.862;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Create mask based on selected regions of interest on point cloud projection
I = double(I);
[m,n,~] = size(I);
polyBW = false([m,n]);
I = reshape(I,[m*n 3]);

% Convert HSV color space to canonical coordinates
Xcoord = I(:,2).*I(:,3).*cos(2*pi*I(:,1));
Ycoord = I(:,2).*I(:,3).*sin(2*pi*I(:,1));
I(:,1) = Xcoord;
I(:,2) = Ycoord;
clear Xcoord Ycoord

% Project 3D data into 2D projected view from current camera view point within app
J = rotateColorSpace(I);

% Apply polygons drawn on point cloud in app
polyBW = applyPolygons(J,polyBW);

% Combine both masks
BW = sliderBW & polyBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end

function J = rotateColorSpace(I)

% Translate the data to the mean of the current image within app
shiftVec = [0.001093 -0.028267 0.338454];
I = I - shiftVec;
I = [I ones(size(I,1),1)]';

% Apply transformation matrix
tMat = [0.987118 -0.788472 0.000000 -0.199014;
    0.529821 1.417374 0.145152 -0.760237;
    0.101132 0.270549 -0.760433 9.024167;
    0.000000 0.000000 0.000000 1.000000];

J = (tMat*I)';
end

function polyBW = applyPolygons(J,polyBW)

% Define each manually generated ROI
hPoints(1).data = [-0.197663 -0.472101;
    -0.205772 -0.609520;
    -0.216586 -0.702610;
    -0.216586 -0.764670;
    -0.284167 -0.879925;
    -0.389595 -0.746939;
    -0.359859 -0.560758];

% Iteratively apply each ROI
for ii = 1:length(hPoints)
    if size(hPoints(ii).data,1) > 2
        in = inpolygon(J(:,1),J(:,2),hPoints(ii).data(:,1),hPoints(ii).data(:,2));
        in = reshape(in,size(polyBW));
        polyBW = polyBW | in;
    end
end

end