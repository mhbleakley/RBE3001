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


  
%Target Positions
Pos1 = [100,-100,130];
Pos2 = [100, 100, 130];

pp.interpolate_jp(pp.ik3001([100,-100,130]),1000);
pause(1.2);

i = 1; % counts iterations
timestep = 1;
y12traj = traj.interpolated_quintic_linear_traj(Pos1(1,2), Pos2(1,2), 0, 3, timestep);

for j = [1:3000]
    z12traj(j) = 30 + abs(sqrt(100-(y12traj(j))^2));
%       disp(130 - 65*(sin(y12traj(j))))
    x12traj(j) = 100;
end


timecount = 1;
timecount2 = 1;
timecount3 = 1;
prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
zeroVector = [0; 0; 0];
angle = pp.measured_js(1,0); 
fkAngle = transpose(angle(1, :));
endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
tic
while toc < 6
    angle = pp.measured_js(1,0); 
    fkAngle = transpose(angle(1, :));
    endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
    
    if ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos2)) && (timecount <= 3000)
            finished_motion = pp.finished_movement(transpose(endpoint(1:3,1)), [x12traj(timecount) y12traj(timecount) z12traj(timecount)]);
          if (~finished_motion)
            pp.interpolate_jp(pp.ik3001([x12traj(timecount), y12traj(timecount), z12traj(timecount)]), 1);
            timecount = timecount + 1;
          end
      
    end
    
    angle = pp.measured_js(1,0); 
    fkAngle = transpose(angle(1, :));
    endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
    %if filters identical data and zero data out to remove noise
    if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) )
        quintic_traj_m(i, 1) = toc;
        quintic_traj_m(i, 2) = angle(1,1);
        quintic_traj_m(i, 3) = angle(1,2);
        quintic_traj_m(i, 4) = angle(1,3);

        quintic_traj_m(i,5) = endpoint(1,1);
        quintic_traj_m(i,6) = endpoint(2,1);
        quintic_traj_m(i,7) = endpoint(3,1);
        i = i+1;
    end
    prevEndpoint = endpoint;
end


% Plotting Traj Data

writematrix(quintic_traj_m,'semicircle_trajectory_data.csv');
filename = 'semicircle_trajectory_data.csv';

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
ylim([-150 150])
zlim([0 150])
  
  
  
  catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc