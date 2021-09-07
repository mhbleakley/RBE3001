%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
% Team 4: Rahil Parikh, Martin Bleakley, Alexander Breiling
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
%   viaPts1 = [0,45,0];
%   x = 1200; 
%   pp.interpolate_jp(viaPts, 000);
%   pause(0.5);
%   pp.interpolate_jp(viaPts1, 0);
%   position = zeros(x, 5);
%   



returnPacket = pp.read(SERVER_ID_READ);
disp(returnPacket);

% Position 1

% 1  -51.1200

% 2   61.6600

% 3  -11.4100

% Position 2

%    -0.4800

%    61.6600

%    30.3500

% Position 3

%    14.4800

%    69.6600

%    -50.3500

% Position 4

%    82.4800

%    54.6600

%    66.3500



  viaPts = [0,0,0];
  viaPts1 = [-51.1200,61.6600,-11.4100];
  viaPts2 = [-0.4800,61.6600,30.3500];
  viaPts3 = [14.4800,69.6600,-50.3500];
  viaPts4 = [82.4800,54.6600,66.3500];
  x = 2000; 
  pp.servo_jp(viaPts);
%   pp.interpolate_jp(viaPts1, x);
%   pp.interpolate_jp(viaPts2, x);
%   pp.interpolate_jp(viaPts3, x);
%   
%   pp.servo_jp(viaPts4);
  pp.interpolate_jp(viaPts4, x);


  n = 0;
  t0 = clock;
  ms = 0;
  ms1 = 0;
  i = 1;
  while ms <= x
%      if ms < ms1 +1
%          ms = round(etime(clock,t0) * 1000);
         
%      else
         ms = etime(clock,t0) * 1000;
         measuredJoints = pp.measured_js(1,0);
         position(i, 1) = ms;
         position(i, 2) = measuredJoints(1, 1);
         position(i, 3) = measuredJoints(1, 2);
         position(i, 4) = measuredJoints(1, 3);
         if i > 1
            position(i, 5) = position(i, 1) - position(i-1, 1);
         end
         ms1 = ms;
         i = i + 1;
%      end
  end

 writematrix(position,'ArmData2.csv');
 filename = 'ArmData1.csv';
 filename2 = 'ArmData2.csv';
 M = csvread(filename);
 N = csvread(filename2);
 time = M(:,1);
 time2 = N(:,1);
 subplot(3,2,1);
 y1 = M(:,2);
 plot(time, y1);
 hold on
 serv1 = N(:,2);
 plot(time2, serv1);
 hold off
 title("Joint 1 Position vs Time");
 xlabel('Time(ms)') ;
 ylabel('Angle(degrees)'); 
 subplot(3,2,2);
 y2 = M(:,3);
 plot(time, y2);
 hold on
 serv2 = N(:,3);
 plot(time2, serv2);
 hold off
 title("Joint 2 Position vs Time");
 xlabel('Time(ms)') ;
 ylabel('Angle(degrees)'); 
 subplot(3,2,3);
 y3 = M(:,4);
 plot(time, y3);
 hold on
 serv3 = N(:,4);
 plot(time2, serv3);
 hold off
 title("Joint 3 Position vs Time");
 xlabel('Time(ms)') ;
 ylabel('Angle(degrees)'); 
 
 subplot(3,2,5);
 A = [M(3:end,5)];
 histogram(A);
 title("Reading increment frequency");
 xlabel('Time(ms)') ;
 ylabel('Frequency(Packets)');
 
 subplot(3,2,6);
 A = [M(3:end,5)];
 histogram(A, 10);
 title("Reading increment frequency (10 bins)");
 xlabel('Time(ms)') ;
 ylabel('Frequency(Packets)');


 % pp.measured_js(1,1);
%   returnPacket = pp.read(SERVER_ID_READ);
%   disp(returnPacket);
%  pp.servo_jp(viaPts);
%   for k = viaPts
%       tic
%       packet = zeros(15, 1, 'single');
%       packet(1) = 0;%one second time
%       packet(2) = 0;%linear interpolation
%       packet(3) = k;
%       packet(4) = 0;% Second link to 0
%       packet(5) = 0;% Third link to 0
% 
%       % Send packet to the server and get the response      
%       %pp.write sends a 15 float packet to the micro controller
%        pp.write(SERV_ID, packet); 
%        %pp.read reads a returned 15 float backet from the micro controller.
%        returnPacket = pp.read(SERVER_ID_READ);
%       toc
% 
%       if DEBUG
%           disp('Sent Packet:')
%           disp(packet);
%           disp('Received Packet:');
%           disp(returnPacket);
%       end
%       
%       toc
%       pause(1) 
%       
%   end
  
  % Closes then opens the gripper
%   pp.closeGripper()
%   pause(1)
%   pp.openGripper()

  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc
