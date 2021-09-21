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

   Point1 = [50,55,75];
   Point2 = [50 0 35];
   Point3 = [140 60 90];
   
%    Pos1 = [50,55,75];
%    Pos2 = [50, 0, 35];
%    Pos3 = [140, 60, 90];
   
   Pos1 = pp.ik3001(Point1);
   Pos2 = pp.ik3001(Point2);
   Pos3 = pp.ik3001(Point3);

   positionArray = [Pos1, Pos2, Pos3, Pos1];
  

% Cubic Trajectory 

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

i = 1; % counts iterations
tic
while toc < 4
    
    if toc >= 0 && toc <= 1
        Point(1,1) = traj.cubic_polynomial(joint1Pos12, toc);
        Point(1,2) = traj.cubic_polynomial(joint2Pos12, toc);
        Point(1,3) = traj.cubic_polynomial(joint3Pos12, toc);
    
    elseif toc > 1 && toc <=2
       Point(1,1) = traj.cubic_polynomial(joint1Pos23, toc);
       Point(1,2) = traj.cubic_polynomial(joint2Pos23, toc);
       Point(1,3) = traj.cubic_polynomial(joint3Pos23, toc);
    
    elseif toc > 2 && toc <= 3
        Point(1,1) = traj.cubic_polynomial(joint1Pos31, toc);
        Point(1,2) = traj.cubic_polynomial(joint2Pos31, toc);
        Point(1,3) = traj.cubic_polynomial(joint3Pos31, toc);
    else
        Point = Pos1;
    end
        
    pp.interpolate_jp(Point, 100);
    
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

% Linear Trajectory 
% 
% i = 1; % counts iterations
% timestep = 1;
% x12traj = traj.linear_traj(Pos1(1,1), Pos2(1,1), 0, 1, timestep);
% x23traj = traj.linear_traj(Pos2(1,1), Pos3(1,1), 1, 2, timestep);
% x31traj = traj.linear_traj(Pos3(1,1), Pos1(1,1), 2, 3, timestep);
% 
% y12traj = traj.linear_traj(Pos1(1,2), Pos2(1,2), 0, 1, timestep);
% y23traj = traj.linear_traj(Pos2(1,2), Pos3(1,2), 1, 2, timestep);
% y31traj = traj.linear_traj(Pos3(1,2), Pos1(1,2), 2, 3, timestep);
% 
% z12traj = traj.linear_traj(Pos1(1,3), Pos2(1,3), 0, 1, timestep);
% z23traj = traj.linear_traj(Pos2(1,3), Pos3(1,3), 1, 2, timestep);
% z31traj = traj.linear_traj(Pos3(1,3), Pos1(1,3), 2, 3, timestep);
% 
% 
% timecount = 1;
% timecount2 = 1;
% timecount3 = 1;
% prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
% zeroVector = [0; 0; 0];
% angle = pp.measured_js(1,0); 
% fkAngle = transpose(angle(1, :));
% endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
% tic
% while toc < 3.2
%     
%     if ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos2)) && (timecount <= 1000)
%       
%           pp.interpolate_jp(pp.ik3001([x12traj(timecount), y12traj(timecount), z12traj(timecount)]), 0);
%           timecount = timecount + 1;
%       
%     elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos3)) && (timecount2 <= 1000)
% 
%           pp.interpolate_jp(pp.ik3001([x23traj(timecount2), y23traj(timecount2), z23traj(timecount2)]), 0);
%           timecount2 = timecount2 + 1;
%             
%     elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos1)) && (timecount3 <= 1000)
%       
%           pp.interpolate_jp(pp.ik3001([x31traj(timecount3), y31traj(timecount3), z31traj(timecount3)]), 0);
%           timecount3 = timecount3 + 1;
% 
%     end
%     
%     angle = pp.measured_js(1,0); 
%     fkAngle = transpose(angle(1, :));
%     endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
%     %if filters identical data and zero data out to remove noise
%     if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
%         lin_traj_m(i, 1) = toc;
%         lin_traj_m(i, 2) = angle(1,1);
%         lin_traj_m(i, 3) = angle(1,2);
%         lin_traj_m(i, 4) = angle(1,3);
% 
%         lin_traj_m(i,5) = endpoint(1,1);
%         lin_traj_m(i,6) = endpoint(2,1);
%         lin_traj_m(i,7) = endpoint(3,1);
%         i = i+1;
%     end
%     prevEndpoint = endpoint;
% end


% Quintic Trajectory

% i = 1; % counts iterations
% timestep = 1;
% x12traj = traj.quintic_linear_traj(Pos1(1,1), Pos2(1,1), 0, 1, timestep);
% x23traj = traj.quintic_linear_traj(Pos2(1,1), Pos3(1,1), 1, 2, timestep);
% x31traj = traj.quintic_linear_traj(Pos3(1,1), Pos1(1,1), 2, 3, timestep);
% 
% y12traj = traj.quintic_linear_traj(Pos1(1,2), Pos2(1,2), 0, 1, timestep);
% y23traj = traj.quintic_linear_traj(Pos2(1,2), Pos3(1,2), 1, 2, timestep);
% y31traj = traj.quintic_linear_traj(Pos3(1,2), Pos1(1,2), 2, 3, timestep);
% 
% z12traj = traj.quintic_linear_traj(Pos1(1,3), Pos2(1,3), 0, 1, timestep);
% z23traj = traj.quintic_linear_traj(Pos2(1,3), Pos3(1,3), 1, 2, timestep);
% z31traj = traj.quintic_linear_traj(Pos3(1,3), Pos1(1,3), 2, 3, timestep);
% 
% timecount = 1;
% timecount2 = 1;
% timecount3 = 1;
% prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
% zeroVector = [0; 0; 0];
% angle = pp.measured_js(1,0); 
% fkAngle = transpose(angle(1, :));
% endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
% tic
% while toc < 3.2
%     
%     if ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos2)) && (timecount <= 1000)
%       
%           pp.interpolate_jp(pp.ik3001([x12traj(timecount), y12traj(timecount), z12traj(timecount)]), 0);
%           timecount = timecount + 1;
%       
%     elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos3)) && (timecount2 <= 1000)
% 
%           pp.interpolate_jp(pp.ik3001([x23traj(timecount2), y23traj(timecount2), z23traj(timecount2)]), 0);
%           timecount2 = timecount2 + 1;
%             
%     elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos1)) && (timecount3 <= 1000)
%       
%           pp.interpolate_jp(pp.ik3001([x31traj(timecount3), y31traj(timecount3), z31traj(timecount3)]), 0);
%           timecount3 = timecount3 + 1;
% 
%     end
%     
%     angle = pp.measured_js(1,0); 
%     fkAngle = transpose(angle(1, :));
%     endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
%     %if filters identical data and zero data out to remove noise
%     if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
%         quintic_traj_m(i, 1) = toc;
%         quintic_traj_m(i, 2) = angle(1,1);
%         quintic_traj_m(i, 3) = angle(1,2);
%         quintic_traj_m(i, 4) = angle(1,3);
% 
%         quintic_traj_m(i,5) = endpoint(1,1);
%         quintic_traj_m(i,6) = endpoint(2,1);
%         quintic_traj_m(i,7) = endpoint(3,1);
%         i = i+1;
%     end
%     prevEndpoint = endpoint;
% end


% Plotting Traj Data

writematrix(traj_m,'trajectory_data.csv');
filename = 'trajectory_data.csv';

% writematrix(lin_traj_m,'linear_trajectory_data.csv');
% filename2 = 'linear_trajectory_data.csv';

% writematrix(quintic_traj_m,'quintic_trajectory_data.csv');
% filename3 = 'quintic_trajectory_data.csv';

traj_data = csvread(filename);
time = traj_data(:,1);

% Position
xPos = traj_data(:,5);
yPos = traj_data(:,6);
zPos = traj_data(:,7);

% Velocity
dx1=gradient(xPos(:))./gradient(time(:));
dy1=gradient(yPos(:))./gradient(time(:));
dz1=gradient(zPos(:))./gradient(time(:));

% Acceleration
dx2=gradient(dx1(:))./gradient(time(:));
dy2=gradient(dy1(:))./gradient(time(:));
dz2=gradient(dz1(:))./gradient(time(:));

% Plot position 

subplot(3,2,1)
plot(time,xPos); 
hold on
plot(time,yPos);
plot(time,zPos);
hold off

title("End Effector Planned Position Trajectory)");
xlabel('Time(s)') ;
ylabel('Position(mm)'); 
legend('X Position', 'Y Position', 'Z Position');

% Plot velocity 

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

% Plot acceleration 

subplot(3,2,3)
plot(time, dx2);
hold on
plot(time, dy2);
plot(time, dz2);
hold off

title("End Effector Planned Acceleration Trajectory)");
xlabel('Time(s)') ;
ylabel('Acceleration(mm/s^2)'); 
legend('X Accleration', 'Y Accleration', 'Z Accleration');

% Plot 3D Trajectory

figure();
plot3(xPos,yPos,zPos);
xlabel('X Position(mm)') ;
ylabel('Y Position(mm)'); 
zlabel('Z Position(mm)'); 
legend('End Effector Path');

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc