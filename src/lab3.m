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
  
%   disp(pp.fk3001([-90;0;0])) % 100 0 195
%   disp(pp.fk3001([20;30;40])) % -14 -32 33
%   disp(pp.fk3001([45;10;18])) % -79 -128 -15
%   disp(pp.fk3001([35;30;30])) % 175 83 140
  
%   viaPts = [0,0,0];
%   Point1 = [35,55,75];
%   Point2 = [100 0 195];
%   Point3 = [-70 -60 90];
%   
% %   pp.interpolate_jp(pp.ik3001(Point1), 2000);
%   pp.plot_arm(pp.ik3001(Point1))
%   pause(6);
% %   pp.interpolate_jp(pp.ik3001(Point2), 2000);
%   pp.plot_arm(pp.ik3001(Point2))
%   pause(6);
% %   pp.interpolate_jp(pp.ik3001(Point3), 2000);
%   pp.plot_arm(pp.ik3001(Point3))
%   pause(6);
%   pp.interpolate_jp(pp.ik3001(Point1), 2000);
%   pp.plot_arm(pp.ik3001(Point1))
%   pause(2.5);
% %   x = 1200; 
%   pp.interpolate_jp(viaPts1, x);
%    checkpoint = pp.fk3001(pp.ik3001([10 30 185]));
%    disp(checkpoint);
%    disp(pp.ik3001([100 0 195]));


% Motion Planning Arm Movement
%    Point1 = [35,55,75];
%    Point2 = [100 0 195];
%    Point3 = [-70 -60 90];
%    
%    Pos1 = pp.ik3001(Point1);
%    Pos2 = pp.ik3001(Point2);
%    Pos3 = pp.ik3001(Point3);
%    
% 
% M = zeros(50000, 7);
% 
% t0 = clock;
% ms = 0;
% ms1 = 0;
% x = 24000;
% i = 1;
% once = 1;
% while ms <= x
%     ms = etime(clock,t0) * 1000;
%     
%     if ms < 3000
%         if(once)
%             pp.interpolate_jp(Pos1, 3000);
%             once = 0;
%         end
%     elseif ms > 3000 && ms < 6000
%         if(once == 0)
%             pp.interpolate_jp(Pos2, 3000);
%             once = 1;
%         end
%         
%     elseif ms > 6000 && ms < 9000
%         if(once)
%             pp.interpolate_jp(Pos3, 3000);
%             once = 0;
%         end
%     elseif ms > 9000 && ms < 12000
%         if(once == 0)
%             pp.interpolate_jp(Pos1, 3000);
%             once = 1;
%         end
%     end
%         
%     angle = pp.measured_js(1,0); 
%     fkAngle = transpose(angle(1, :));
%     M(i, 1) = ms;
%     M(i, 2) = angle(1,1);
%     M(i, 3) = angle(1,2);
%     M(i, 4) = angle(1,3);
%     endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
%     M(i,5) = endpoint(1,1);
%     M(i,6) = endpoint(2,1);
%     M(i,7) = endpoint(3,1);
%     
%     
%     i = i + 1;
% end
% 
% writematrix(M,'IKvalidation.csv');

filename = 'IKvalidation.csv';
motionData = csvread(filename);
time = motionData(:,1);
% xPos = motionData(:,5);
% yPos = motionData(:,6);
% zPos = motionData(:,7);
% 
% plot(time,xPos);
% hold on
% plot(time,yPos);
% plot(time,zPos);
% hold off
% 
% title("End Effector position vs Time");
% xlabel('Time(ms)') ;
% ylabel('Position(mm)'); 
% legend('X Position', 'Y Position', 'Z Position');

% plot3(xPos,yPos,zPos);
% xlabel('X Position(mm)') ;
% ylabel('Y Position(mm)'); 
% zlabel('Z Position(mm)'); 
% legend('End Effector Path');


joint1 = motionData(:,2);
joint2 = motionData(:,3);
joint3 = motionData(:,4);

plot(time, joint1);
hold on
plot(time, joint2);
plot(time, joint3);
hold off
title("Joint Angles vs Time");
xlabel('Time(ms)') ;
ylabel('Angle(degrees)'); 
legend('Joint1', 'Joint2', 'Joint3');

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc