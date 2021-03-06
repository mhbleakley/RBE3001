classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
             packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(packet, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(packet, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(packet.GRIPPER_ID);
                packet.myHIDSimplePacketComs.writeBytes(intid, ds, packet.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
%         function pos = readGripper(packet)
%             try
%                 ds = javaArray('java.lang.Byte',length(1));
%                 intid = java.lang.Integer(packet.GRIPPER_ID);
%                 pos = packet.myHIDSimplePacketComs.readBytes(intid, ds);
%             catch exception
%                 getReport(exception)
%                 disp('Command error, reading too fast');
%             end
%         end
        
        % Opens the gripper
        function openGripper(packet)
            packet.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(packet)
            packet.writeGripper(0);
        end
        
        function packet = measured_js(self, GETPOS, GETVEL)
            packet = zeros(2, 3, 'single');
          %  packet1 = zeros(1,9, 'single');
            if GETPOS
                SERVER_ID_READ =1910;
                returnPacket = self.read(SERVER_ID_READ);
                packet(1,1) = returnPacket(3);
                packet(1,2) = returnPacket(5);
                packet(1,3) = returnPacket(7);
            end
            if GETVEL
                SERVER_ID_READ =1822;
                returnPacket = self.read(SERVER_ID_READ);
                packet(2,1) = returnPacket(3);
                packet(2,2) = returnPacket(6);
                packet(2,3) = returnPacket(9);
            end
            %disp(packet);
        end
        
        function packet = setpoint_js(self)
            packet = zeros(1, 3, 'single');
                SERVER_ID_READ =1910;
                returnPacket = self.read(SERVER_ID_READ);
                packet(1,1) = returnPacket(3);
                packet(1,2) = returnPacket(4);
                packet(1,3) = returnPacket(5);
            disp(packet);
        end
        
        
        function packet = goal_js(self)
            packet = zeros(1, 3, 'single');
                SERVER_ID_READ =1848;
                returnPacket = self.read(SERVER_ID_READ);
                packet(1,1) = returnPacket(3);
                packet(1,2) = returnPacket(4);
                packet(1,3) = returnPacket(5);
            disp(packet);
        end
        
        function interpolate_jp(self, values, int)
            SERV_ID = 1848;
            packet = zeros(15, 1, 'single');
            packet(1) = int;%one second time
            packet(2) = 0;%linear interpolation
            packet(3) = values(1);
            packet(4) = values(2);% Second link to 0
            packet(5) = values(3);% Third link to 0
            % Send packet to the server and get the response      
            %pp.write sends a 15 float packet to the micro controller
            self.write(SERV_ID, packet);

        end
        
        function time = countMs(self)
            
        end
        
        % Moves servos to specific angles
        function servo_jp(self, values)
            SERV_ID = 1848;
            packet = zeros(15, 1, 'single');
            packet(1) = 0;% One second time
            packet(2) = 0;% Linear interpolation
            packet(3) = values(1);% First link
            packet(4) = values(2);% Second link
            packet(5) = values(3);% Third link

            % Send packet to the server and get the response      
            %pp.write sends a 15 float packet to the micro controller
            self.write(SERV_ID, packet); 
        end
        
        function T = trotz(self, theta)
            T = [cos(theta) -sin(theta) 0 0;
            sin(theta) cos(theta) 0 0;
            0 0 1 0;
            0 0 0 1;];
        end

        function T = trotx(self, theta)
            T = [1 0 0 0;
            0 cos(theta) -sin(theta) 0;
            0 sin(theta) cos(theta) 0;
            0 0 0 1;];
        end

        function T = troty(self, theta)
            T = [cos(theta) 0 -sin(theta) 0;
            0 1 0 0;
            sin(theta) 0 cos(theta) 0;
            0 0 0 1;];
        end
        
        function T = dh2mat(self, q)
            T = self.trotz(q(1)) * [1 0 0 0;
                          0 1 0 0;
                          0 0 1 q(2);
                          0 0 0 1;] * [1 0 0 q(3);
                                       0 1 0 0;
                                       0 0 1 0;
                                       0 0 0 1;] * self.trotx(q(4));
                                   %disp(T)
        end
        
        function T = dh2fk(self, q)

            T = eye(4,4);
  
            
            s0 = size(q); % size of row by col matrix
            
            n = s0(1); % number of rows in q
            M = cell(n, n);
%             i = 0;
            for i = [1:n]
                row = q(i,:);
                %disp(row);
                M{i} = self.dh2mat(row);
%                 disp(M{i})
                
            end

            a = size(M);
            b = a(1);
            for j = [1:b]
                %disp(T);
                T = T * M{j};
                disp(T);
                 
            end

            %disp(T)
            
        end
        
        function T= fk3001(self, q)
            q = q*pi/180;
            T = [ cos(q(1))*cos(q(2) - pi/2)*cos(q(3) + pi/2) - cos(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2), - cos(q(1))*cos(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(1))*cos(q(3) + pi/2)*sin(q(2) - pi/2), -sin(q(1)), 100*cos(q(1))*cos(q(2) - pi/2) + 100*cos(q(1))*cos(q(2) - pi/2)*cos(q(3) + pi/2) - 100*cos(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2);
                 cos(q(2) - pi/2)*cos(q(3) + pi/2)*sin(q(1)) - sin(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2), - cos(q(2) - pi/2)*sin(q(1))*sin(q(3) + pi/2) - cos(q(3) + pi/2)*sin(q(1))*sin(q(2) - pi/2),  cos(q(1)), 100*cos(q(2) - pi/2)*sin(q(1)) + 100*cos(q(2) - pi/2)*cos(q(3) + pi/2)*sin(q(1)) - 100*sin(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2);
                                       - cos(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(3) + pi/2)*sin(q(2) - pi/2),                           sin(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(2) - pi/2)*cos(q(3) + pi/2),            0,                                95 - 100*cos(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(3) + pi/2)*sin(q(2) - pi/2) - 100*sin(q(2) - pi/2);
                                                                                                                    0,                                                                                                       0,            0,                                                                                                                                                  1];
%             disp(T);
            
        end
        
        function T = measured_cp(self)
            M = zeros(3,1);
            N = zeros(2,3);
            N = self.measured_js(1,0)*(2*pi/360);
            M(1,1) = N(1,1);
            M(2,1) = N(1,2);
            M(3,1) = N(1,3);
            T = self.fk3001(M);
        end
        
        function T = setpoint_cp(self)
            M = zeros(3,1);
            N = zeros(2,3);
            N = self.setpoint_js()*(2*pi/360);
            M(1,1) = N(1,1);
            M(2,1) = N(1,2);
            M(3,1) = N(1,3);
            T = self.fk3001(M);
        end

            function T = goal_cp(self)
            M = zeros(3,1);
            N = zeros(2,3);
            N = self.goal_js()*(2*pi/360);
            M(1,1) = N(1,1);
            M(2,1) = N(1,2);
            M(3,1) = N(1,3);
            T = self.fk3001(M);
            end
        
            function plot_arm(self,q,velocity)
                q = q*(pi/180);
                
                T01 =   [ 1, 0, 0,  0;
                         0, 1, 0,  0;
                         0, 0, 1, 55;
                         0, 0, 0,  1];
                
                T12 =   [ cos(q(1)),  0, -sin(q(1)),  0;
                         sin(q(1)),  0,  cos(q(1)),  0;
                                   0, -1,            0, 40;
                                   0,  0,            0,  1];

                T23 =   [ cos(q(2) - pi/2), -sin(q(2) - pi/2), 0, 100*cos(q(2) - pi/2);
                         sin(q(2) - pi/2),  cos(q(2) - pi/2), 0, 100*sin(q(2) - pi/2);
                                          0,                   0, 1,                      0;
                                          0,                   0, 0,                      1];


                T34 =    [cos(q(3) + pi/2), -sin(q(3) + pi/2), 0, 100*cos(q(3) + pi/2);
                         sin(q(3) + pi/2),  cos(q(3) + pi/2), 0, 100*sin(q(3) + pi/2);
                                          0,                   0, 1,                      0;
                                          0,                   0, 0,                      1];

                 Point0 = [0; 0; 0; 1];
                 Point1 = T01*Point0;
                 Point2 = (T01*T12)*Point0; 
                 Point3 = (T01*T12*T23)*Point0;
                 Point4 = (T01*T12*T23*T34)*Point0;
                 
                 X = [Point0(1,1) Point1(1,1) Point2(1,1) Point3(1,1) Point4(1,1)];
                 Y = [Point0(2,1) Point1(2,1) Point2(2,1) Point3(2,1) Point4(2,1)];
                 Z = [Point0(3,1) Point1(3,1) Point2(3,1) Point3(3,1) Point4(3,1)];


                 plot3(X, Y, Z, '-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;
                 xlabel('X Axis');
                 ylabel('Y Axis');
                 zlabel('Z Axis');
                 axis([-200 200 -200 200 0 200]);
                   
%                  Base frame (frame 0)
                 line([X(1),X(1)+50],[Y(1),Y(1)],[Z(1),Z(1)], 'Color', 'r', 'LineWidth', 2);
                 line([X(1),X(1)],[Y(1),Y(1)+50],[Z(1),Z(1)], 'Color', 'b', 'LineWidth', 2);
                 line([X(1),X(1)],[Y(1),Y(1)],[Z(1),Z(1)+50], 'Color', 'g', 'LineWidth', 2);

%                  Frame 1
                 line([X(2),X(2)+50],[Y(2),Y(2)],[Z(2),Z(2)], 'Color', 'r', 'LineWidth', 2);
                 line([X(2),X(2)],[Y(2),Y(2)+50],[Z(2),Z(2)], 'Color', 'b', 'LineWidth', 2);
                 line([X(2),X(2)],[Y(2),Y(2)],[Z(2),Z(2)+50], 'Color', 'g', 'LineWidth', 2);
                 
%                   Frame 2
                 line([X(3),X(3)+50],[Y(3),Y(3)],[Z(3),Z(3)], 'Color', 'r', 'LineWidth', 2);
                 line([X(3),X(3)],[Y(3),Y(3)+50],[Z(3),Z(3)], 'Color', 'g', 'LineWidth', 2);
                 line([X(3),X(3)],[Y(3),Y(3)],[Z(3),Z(3)+50], 'Color', 'b', 'LineWidth', 2);
                 
%                   Frame 3
                 line([X(4),X(4)+50],[Y(4),Y(4)],[Z(4),Z(4)], 'Color', 'b', 'LineWidth', 2);
                 line([X(4),X(4)],[Y(4),Y(4)+50],[Z(4),Z(4)], 'Color', 'g', 'LineWidth', 2);
                 line([X(4),X(4)],[Y(4),Y(4)],[Z(4),Z(4)+50], 'Color', 'r', 'LineWidth', 2);
                 
%                   End effector frame (frame 4)
                 line([X(5),X(5)+50],[Y(5),Y(5)],[Z(5),Z(5)], 'Color', 'r', 'LineWidth', 2);
                 line([X(5),X(5)],[Y(5),Y(5)+50],[Z(5),Z(5)], 'Color', 'g', 'LineWidth', 2);
                 line([X(5),X(5)],[Y(5),Y(5)],[Z(5),Z(5)+50], 'Color', 'b', 'LineWidth', 2);
                 
%                  line([X(5),X(5)+velocity(1)/50],[Y(5),Y(5)+velocity(1)/50],[Z(5),Z(5)+velocity(1)/50], 'Color', 'magenta', 'LineWidth', 2);
                 %Velocity Vector
                 hold on
                 quiver3(X(5),Y(5),Z(5),(velocity(1)/50),velocity(2)/50,velocity(1)/50)
                 hold off
                 legend('arm','x-axis', 'y-axis', 'z-axis');
                 drawnow;
            end
            
            function Q = ik3001(self, pos)

            a1 = 100;
            a2 = 100; 
            d1 = 95;
            x = pos(1);
            y = pos(2);
            z = pos(3);
            r = sqrt((x^2) + (y^2));
            s = z - d1;
            
            D1 = x/r;
            C1 = sqrt(1 - (D1^2));
            theta1 = atan2(y,x);
            
            alpha = atan2(s,r);
            D2 = ((a1^2) + (r^2) + (s^2)- (a2^2))/(2*a1*(sqrt((r^2) + ( s^2))));

            C2 = -(sqrt(1 - (D2^2)));
            beta = atan2(C2,D2);
            theta2 = pi/2 - (alpha - beta);
            
            D3 = -((a1^2) + (a2^2) - ((r^2) + (s^2)))/(2*a1*a2);
            C3 = sqrt(1 - (D3^2));
            theta3 =  (atan2(C3,D3))- pi/2;
            
            if theta2 <= -pi/2 || theta2 >= pi/2
                theta2 = pi/2 - (alpha - (atan2(-C2, D2))); % safety checks for angles 2 and 3
            end
            if theta3 <= -pi/2 || theta3 >= pi/2
                theta3 = atan2(-C3, D3) - pi/2;
            end
            
            Q = [theta1, theta2, theta3]; 
            Q = Q * (360/(2*pi));
            
            end
            
            function B = finished_movement(self,currPos, targetPos)
                
%                 currentPV = self.measured_js(1,0);
                currentPos = currPos;
                
                if(abs(currentPos(1,1)) >= abs(targetPos(1,1))-.5 && abs(currentPos(1,2)) >= abs(targetPos(1,2))-.5 && abs(currentPos(1,3)) >= abs(targetPos(1,3))-.5)
                    B1 = 1;
                else 
                    B1 = 0;
                end
                
                if(abs(currentPos(1,1)) <= abs(targetPos(1,1)) + .5 && abs(currentPos(1,2))+1 <= abs(targetPos(1,2)) + .5 && abs(currentPos(1,3)) <= abs(targetPos(1,3))+.5)
                    B2 = 1;
                else 
                    B2 = 0;
                end
                
                B = B1*B2;
                %                 disp(self.readGripper());
%                 pause(5);

                
            end
            
            function J = jacob3001(self, q)
                q = q*pi/180;
                J11 = 100*sin(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(2) - pi/2)*cos(q(3) + pi/2)*sin(q(1)) - 100*cos(q(2) - pi/2)*sin(q(1));
                J21 = 100*cos(q(1))*cos(q(2) - pi/2) + 100*cos(q(1))*cos(q(2) - pi/2)*cos(q(3) + pi/2) - 100*cos(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2);
                J31 = 0;
                
                J12 = - 100*cos(q(1))*sin(q(2) - pi/2) - 100*cos(q(1))*cos(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(1))*cos(q(3) + pi/2)*sin(q(2) - pi/2);
                J22 = - 100*sin(q(1))*sin(q(2) - pi/2) - 100*cos(q(2) - pi/2)*sin(q(1))*sin(q(3) + pi/2) - 100*cos(q(3) + pi/2)*sin(q(1))*sin(q(2) - pi/2);
                J32 = 100*sin(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(2) - pi/2)*cos(q(3) + pi/2) - 100*cos(q(2) - pi/2);
                
                J13 = - 100*cos(q(1))*cos(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(1))*cos(q(3) + pi/2)*sin(q(2) - pi/2);
                J23 = - 100*cos(q(2) - pi/2)*sin(q(1))*sin(q(3) + pi/2) - 100*cos(q(3) + pi/2)*sin(q(1))*sin(q(2) - pi/2);
                J33 = 100*sin(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(2) - pi/2)*cos(q(3) + pi/2);
                
                J41 = 0;
                J51 = 0;
                J61 = 1;
                
                J42 = -sin(q(1));
                J52 = cos(q(1));
                J62 = 0;
                
                J43 = -sin(q(1));
                J53 = cos(q(1));
                J63 = 0;
                
                J = [J11 J12 J13;
                    J21 J22 J23;
                    J31 J32 J33;
                    J41 J42 J43;
                    J51 J52 J53;
                    J61 J62 J63;];
%                 disp(J);
            end
            
            function PD = fdk3001(self, q, qd)
                PD = self.jacob3001(q)*qd;
            end
            
            function goto_ball(self,checkPoint,endPoint)
                zeroPoint = [100, 0, 195];
                traj2 = Traj_Planner();
                
                tic
                while toc < 5.2
                    if ~(self.finished_movement(zeroPoint, checkPoint)) && (toc <= 3)
                        x = traj2.linear_traj(100, checkPoint(1), 0, 3, toc);
                        y = traj2.linear_traj(0, checkPoint(2), 0, 3, toc);
                        self.interpolate_jp(self.ik3001([x, y, 195]),0);
                    elseif ~(self.finished_movement(zeroPoint, checkPoint)) && toc > 3 && toc <= 5
                        z = traj2.linear_traj(195, checkPoint(3), 3, 5, toc);
                        self.interpolate_jp(self.ik3001([x, y, z]),0);
                    end
                end
                
                tic
%                 while toc < 3
%                     angle = self.measured_js(1,0);
%                     fkAngle = transpose(angle(1, :));
%                     endpoint = self.fk3001(fkAngle)* [0; 0; 0; 1];
%                     endAnglePoint = self.ik3001(endpoint(1:3, 1));
%                     endAnglePoint(1,3) = endAnglePoint(1,3) - 5;
%                     self.interpolate_jp(endAnglePoint, 2000);
%                     
%                 end
                self.closeGripper();
%                 disp(self.readGripper());
%                 pause(5);
                self.interpolate_jp(self.ik3001(zeroPoint), 2000);
                pause(2.5);
                self.interpolate_jp(endPoint, 2000);
                pause(2.5);
                self.openGripper();
                pause(1);
                self.interpolate_jp(self.ik3001(zeroPoint), 2000);
                pause(2.5);
                
            end
            
    end
end
