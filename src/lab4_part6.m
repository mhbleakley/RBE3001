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
  
% Linear Trajectory 
Pos1 = [50,55,75];
Pos2 = [100 0 195];

pp.interpolate_jp(pp.ik3001(Pos1),1000);
% pp.interpolate_jp([100 0 295],1000);
pause(1)

i = 1; % counts iterations
prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
AV = pp.measured_js(1,1); 
fkAngle = transpose(AV(1, :));
qVelocity = AV(2,:);
endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
eStop = 0;
ee_jacob = pp.jacob3001(fkAngle);
detJ = det(ee_jacob(1:3,:))
x12traj = -500;
y12traj = -500
z12traj = -500

tic
while toc < 5 && eStop == 0;
    
            ee_jacob = pp.jacob3001(fkAngle)
            detJ = det(ee_jacob(1:3,:))
            if abs(detJ) <= 100
                eStop = 1;
                disp("Singularity Detected")
                
            end
    
    if ~(pp.finished_movement(transpose(endpoint(1:3,1)), [x12traj y12traj z12traj])) && (toc <= 3) && eStop == 0;
            
        if(eStop == 0)    
        x12traj = traj.linear_traj(Pos1(1,1), Pos2(1,1), 0, 3, toc)  
        y12traj = traj.linear_traj(Pos1(1,2), Pos2(1,2), 0, 3, toc)
        z12traj = traj.linear_traj(Pos1(1,3), Pos2(1,3), 0, 3, toc)
        pp.interpolate_jp(pp.ik3001([x12traj, y12traj, z12traj]),10);
        end
    end
    
    AV = pp.measured_js(1,1);
    fkAngle = transpose(AV(1, :));
    qVelocity = transpose(AV(2,:));
    endpoint = pp.fk3001(fkAngle)* [0; 0; 0; 1];
%     ee_jacob = pp.jacob3001(fkAngle);
%     detJ = det(ee_jacob(1:3,:))
    ee_velocity = ee_jacob(1:3,1:3)*qVelocity;
    pp.plot_arm(fkAngle,ee_velocity)
    
    if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
        lin_traj_m(i, 1) = toc;
        lin_traj_m(i, 2) = AV(2,1);
        lin_traj_m(i, 3) = AV(2,2);
        lin_traj_m(i, 4) = AV(2,3);

        lin_traj_m(i,5) = ee_velocity(1,1);
        lin_traj_m(i,6) = ee_velocity(2,1);
        lin_traj_m(i,7) = ee_velocity(3,1);
        lin_traj_m(i,8) = sqrt(ee_velocity(1,1)^2 + ee_velocity(2,1)^2 + ee_velocity(3,1)^2); %Calculates Scalar Velocity
        i = i+1;
    end

    prevEndpoint = endpoint;
    
end
  
  
  catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc