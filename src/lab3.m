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
traj = Traj_Planner();
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
%    checkpoint = pp.fk3001(pp.ik3001([72 80 120]));
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
% t0 = clock;
% ms = 0;
% ms1 = 0;
% x = 10000;
% i = 1;
% once = 1;
% while ms <= x
%     ms = etime(clock,t0) * 1000;
%     
% %     if ms < 3000
% %         if(once)
% %             pp.interpolate_jp(Pos1, 3000);
% %             once = 0;
% %         end
%     if  ms < 3000
%         if(once)
%             pp.interpolate_jp(Pos2, 3000);
%             once = 0;
%         end
%         
%     elseif ms > 3000 && ms < 6000
%         if(once == 0)
%             pp.interpolate_jp(Pos3, 3000);
%             once = 1;
%         end
%     elseif ms > 6000 && ms < 9000
%         if(once)
%             pp.interpolate_jp(Pos1, 3000);
%             once = 0;
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
% 
% traj.cubic_traj(0,3,0,0,35,100);
% 
% filename = 'IKvalidation.csv';
% motionData = csvread(filename);
% time = motionData(:,1);
% 
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
% 
% figure();
% plot3(xPos,yPos,zPos);
% xlabel('X Position(mm)') ;
% ylabel('Y Position(mm)'); 
% zlabel('Z Position(mm)'); 
% legend('End Effector Path');
% 
% figure();
% joint1 = motionData(:,2);
% joint2 = motionData(:,3);
% joint3 = motionData(:,4);
% 
% plot(time, joint1);
% hold on
% plot(time, joint2);
% plot(time, joint3);
% hold off
% title("Joint Angles vs Time");
% xlabel('Time(ms)') ;
% ylabel('Angle(degrees)'); 
% legend('Joint1', 'Joint2', 'Joint3');

   Point1 = [35,55,75];
   Point2 = [100 0 195];
   Point3 = [-70 -60 90];
   
   Pos1 = pp.ik3001(Point1);
   Pos2 = pp.ik3001(Point2);
   Pos3 = pp.ik3001(Point3);

   positionArray = [Pos1, Pos2, Pos3, Pos1];
%    
%    Pos 1-> 2 : 0.0479
%    Pos 2-> 3 : 0.0439
%    Pos 3-> 4 : 0.1295
%   
%    tic
%    pp.servo_jp(Pos1);
%    currentAV = pp.measured_js(1,0);
%    currentAngle = currentAV(1,:);
%    while currentAngle(1,1)+2 <= Pos1(1,1) &&  currentAngle(1,2)+2 <= Pos1(1,2)
%         currentAV = pp.measured_js(1,0);
%         currentAngle = currentAV(1,:);
%    end
%    disp(toc);
% 

pp.interpolate_jp(Pos1, 2000); % Go to starting Position
pause(2.2);
joint1Pos12 = traj.cubic_traj(0,1,0,0,Pos1(1,1), Pos2(1,1));
joint2Pos12 = traj.cubic_traj(0,1,0,0,Pos1(1,2), Pos2(1,2));
joint3Pos12 = traj.cubic_traj(0,1,0,0,Pos1(1,3), Pos2(1,3));

joint1Pos23 = traj.cubic_traj(1,2,0,0,Pos2(1,1), Pos3(1,1));
joint2Pos23 = traj.cubic_traj(1,2,0,0,Pos2(1,2), Pos3(1,2));
joint3Pos23 = traj.cubic_traj(1,2,0,0,Pos2(1,3), Pos3(1,3));

joint1Pos31 = traj.cubic_traj(2,3,0,0,Pos3(1,1), Pos1(1,1));
joint2Pos31 = traj.cubic_traj(2,3,0,0,Pos3(1,2), Pos1(1,2));
joint3Pos31 = traj.cubic_traj(2,3,0,0,Pos3(1,3), Pos1(1,3));

finished1 = 0;
finished2 = 0;
finished3 = 0;
i = 1; % counts iterations
tic
while toc < 4
    
    if toc >= 0 && toc <= 1
        Point(1,1) = traj.cubic_polynomial(joint1Pos12(1,1), joint1Pos12(2,1), joint1Pos12(3,1), joint1Pos12(4,1), toc);
        Point(1,2) = traj.cubic_polynomial(joint2Pos12(1,1), joint2Pos12(2,1), joint2Pos12(3,1), joint2Pos12(4,1), toc);
        Point(1,3) = traj.cubic_polynomial(joint3Pos12(1,1), joint3Pos12(2,1), joint3Pos12(3,1), joint3Pos12(4,1), toc);
    
    elseif toc > 1 && toc <=2
       Point(1,1) = traj.cubic_polynomial(joint1Pos23(1,1), joint1Pos23(2,1), joint1Pos23(3,1), joint1Pos23(4,1), toc);
       Point(1,2) = traj.cubic_polynomial(joint2Pos23(1,1), joint2Pos23(2,1), joint2Pos23(3,1), joint2Pos23(4,1), toc);
       Point(1,3) = traj.cubic_polynomial(joint3Pos23(1,1), joint3Pos23(2,1), joint3Pos23(3,1), joint3Pos23(4,1), toc);
    
    elseif toc > 2 && toc <= 3
        Point(1,1) = traj.cubic_polynomial(joint1Pos31(1,1), joint1Pos31(2,1), joint1Pos31(3,1), joint1Pos31(4,1), toc);
        Point(1,2) = traj.cubic_polynomial(joint2Pos31(1,1), joint2Pos31(2,1), joint2Pos31(3,1), joint2Pos31(4,1), toc);
        Point(1,3) = traj.cubic_polynomial(joint3Pos31(1,1), joint3Pos31(2,1), joint3Pos31(3,1), joint3Pos31(4,1), toc);
    else
        Point = Pos1;
    end
        
    
    if(finished1 == 0)
        pp.interpolate_jp(Point, 100);    
    end
    if(pp.finished_movement(Point, Pos2) && finished1 == 0)
        finished1 = 1;
    end
    
    if finished1 == 1 && finished2 == 0
        pp.interpolate_jp(Point, 100);
    end
    
    if(pp.finished_movement(Point, Pos3) && finished1 == 1)
        finished2 = 1;
    end
    
    if(finished2 == 1)
        pp.interpolate_jp(Point, 100)
    end    
    
    if(pp.finished_movement(Point, Pos1) && finished2 == 1)
        finished2 = 2;
    end
    
    angle = pp.measured_js(1,0); 
    fkAngle = transpose(angle(1, :));
    traj_m(i, 1) = toc;
    traj_m(i, 2) = angle(1,1);
    traj_m(i, 3) = angle(1,2);
    traj_m(i, 4) = angle(1,3);
    endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
    traj_m(i,5) = endpoint(1,1);
    traj_m(i,6) = endpoint(2,1);
    traj_m(i,7) = endpoint(3,1);
    i = i+1;
    
 
end

%Plotting Traj Data
writematrix(traj_m,'trajectory_data.csv');
filename = 'trajectory_data.csv';
traj_data = csvread(filename);
time = traj_data(:,1);

xPos = traj_data(:,5);
yPos = traj_data(:,6);
zPos = traj_data(:,7);

subplot(3,2,1)
plot(time,xPos); 
dx1=gradient(xPos(:))./gradient(time(:))
hold on
plot(time,yPos);
dy1=gradient(yPos(:))./gradient(time(:))
plot(time,zPos);
dz1=gradient(zPos(:))./gradient(time(:))
hold off

title("End Effector Planned Position Trajectory)");
xlabel('Time(s)') ;
ylabel('Position(mm)'); 
legend('X Position', 'Y Position', 'Z Position');

subplot(3,2,2)
plot(time, dx1);
hold on
plot(time, dy1);
plot(time, dz1);
hold off

title("End Effector Planned Velocity Trajectory)");
xlabel('Time(s)') ;
ylabel('Velocity(mm/s)'); 
legend('X Velocity', 'Y Velocity', 'Z Velocity');


catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc