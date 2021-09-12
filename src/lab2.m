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

% pp.dh2mat([pi 5 5 pi])
syms theta1 theta2 theta3 theta4 d a
%pp.dh2fk([[0 55 0 0; theta1 40 0 -pi/2; (theta2 -pi/2) 0 100 0; (theta3+pi/2) 0 100 0]])
% pp.fk3001([0 0 0]);

% M = cell(10, 1);
% for i = [1 : 10] 
%     
%     pp.interpolate_jp([15,20,30],1000);
%     pause(2);
%     pp.servo_jp([0,0,0]);
%     pause(0.5);
%     M{i} = pp.measured_cp();
% 
% end

% for a = [1:10]
%     disp(M{a})
% end
% 
% writecell(M,'home_position.csv');
filename = 'home_position.csv';
% N = cell(10,1);
N = csvread(filename);

O = zeros(10,3);

for i = [1:10]
    O(i, 1) = N(i, 13);
    O(i, 2) = N(i, 14);
    O(i, 3) = N(i, 15);
end 

% disp(O)
% 
% for i = [1:10]
%     plot3(O(i,1),O(i,2),O(i,3), 'o');
%     hold on
% end
% hold off

% tic
% while toc < 1.5
%   pp.goal_cp()
%   pp.setpoint_cp();
%     pp.measured_cp()
% end

mean_value = mean(O);
rms_value = rms(O);

disp(mean_value)
disp(rms_value)

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc
