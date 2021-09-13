clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 

%   viaPts = [0,0,0];
%   viaPts1 = [35,55,75];
%   x = 1200; 
%   pp.interpolate_jp(viaPts1, x);
%   pause(0.5);
%   pp.interpolate_jp(viaPts1, 0);
%   position = zeros(x, 5);
%   

%     angles = pp.measured_cp;
%     pp.plot_arm(angles);
%     drawnow;

% pp.interpolate_jp([10,45,-20], 6000); % Part 7 Pos1
% pp.interpolate_jp([-35,40,20], 6000); % Part 7 Pos 2
% pp.interpolate_jp([60,60,30], 6000); % Part 7 Pos 3
% pp.interpolate_jp([13,13,13], 6000); % Part 7 Pos 4
% pp.interpolate_jp([90,0,0], 6000); % Part 7 Pos 5

tic
while toc<8
if mod(toc,1)
    angles = pp.measured_js(1,0);
    newAngles = angles(1,:)*2*pi/360;
    disp(newAngles)
    pp.plot_arm(transpose(newAngles)); drawnow
end
    
end

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc