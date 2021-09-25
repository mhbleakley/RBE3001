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
  
  J = pp.jacob3001([0 90 -90]);
  disp(det(J(1:3,:)));
  
  
   Pos1 = [50,55,75];
   Pos2 = [50 0 35];
   Pos3 = [140 60 90];
  
% Linear Trajectory 

i = 1; % counts iterations
timestep = 1;
x12traj = traj.linear_traj(Pos1(1,1), Pos2(1,1), 0, 1, timestep);
x23traj = traj.linear_traj(Pos2(1,1), Pos3(1,1), 1, 2, timestep);
x31traj = traj.linear_traj(Pos3(1,1), Pos1(1,1), 2, 3, timestep);

y12traj = traj.linear_traj(Pos1(1,2), Pos2(1,2), 0, 1, timestep);
y23traj = traj.linear_traj(Pos2(1,2), Pos3(1,2), 1, 2, timestep);
y31traj = traj.linear_traj(Pos3(1,2), Pos1(1,2), 2, 3, timestep);

z12traj = traj.linear_traj(Pos1(1,3), Pos2(1,3), 0, 1, timestep);
z23traj = traj.linear_traj(Pos2(1,3), Pos3(1,3), 1, 2, timestep);
z31traj = traj.linear_traj(Pos3(1,3), Pos1(1,3), 2, 3, timestep);


timecount = 1;
timecount2 = 1;
timecount3 = 1;
prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
zeroVector = [0; 0; 0];
AV = pp.measured_js(1,1); 
fkAngle = transpose(AV(1, :));
qVelocity = AV(2,:);
endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
tic
while toc < 6
    
    if ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos2)) && (timecount <= 1000)
      
          pp.interpolate_jp(pp.ik3001([x12traj(timecount), y12traj(timecount), z12traj(timecount)]), 0);
          timecount = timecount + 1;
      
    elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos3)) && (timecount2 <= 1000)

          pp.interpolate_jp(pp.ik3001([x23traj(timecount2), y23traj(timecount2), z23traj(timecount2)]), 0);
          timecount2 = timecount2 + 1;
            
    elseif ~(pp.finished_movement(transpose(endpoint(1:3,1)), Pos1)) && (timecount3 <= 1000)
      
          pp.interpolate_jp(pp.ik3001([x31traj(timecount3), y31traj(timecount3), z31traj(timecount3)]), 0);
          timecount3 = timecount3 + 1;

    end
    
    AV = pp.measured_js(1,1); 
    fkAngle = transpose(AV(1, :));
    qVelocity = AV(2,:);
    endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
    %if filters identical data and zero data out to remove noise
    if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
        lin_traj_m(i, 1) = toc;
        lin_traj_m(i, 2) = AV(1,1);
        lin_traj_m(i, 3) = AV(1,2);
        lin_traj_m(i, 4) = AV(1,3);

        lin_traj_m(i,5) = endpoint(1,1);
        lin_traj_m(i,6) = endpoint(2,1);
        lin_traj_m(i,7) = endpoint(3,1);
        i = i+1;
    end
    prevEndpoint = endpoint;
%     pp.plot_arm(fkAngle);
%     drawnow;
end

  
  catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc