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
                packet(2,1) = returnPacket(2);
                packet(2,2) = returnPacket(5);
                packet(2,3) = returnPacket(8);
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
                
            end

            a = size(M);
            b = a(1);
            for j = [1:b]
                %disp(T);
                T = T * M{j};
                %disp(T);
                 
            end

            %disp(T)
            
        end
        
        function T= fk3001(self, q)
            T = [ cos(q(1))*cos(q(2) - pi/2)*cos(q(3) + pi/2) - cos(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2), - cos(q(1))*cos(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(1))*cos(q(3) + pi/2)*sin(q(2) - pi/2), -sin(q(1)), 100*cos(q(1))*cos(q(2) - pi/2) + 100*cos(q(1))*cos(q(2) - pi/2)*cos(q(3) + pi/2) - 100*cos(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2);
                 cos(q(2) - pi/2)*cos(q(3) + pi/2)*sin(q(1)) - sin(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2), - cos(q(2) - pi/2)*sin(q(1))*sin(q(3) + pi/2) - cos(q(3) + pi/2)*sin(q(1))*sin(q(2) - pi/2),  cos(q(1)), 100*cos(q(2) - pi/2)*sin(q(1)) + 100*cos(q(2) - pi/2)*cos(q(3) + pi/2)*sin(q(1)) - 100*sin(q(1))*sin(q(2) - pi/2)*sin(q(3) + pi/2);
                                       - cos(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(3) + pi/2)*sin(q(2) - pi/2),                           sin(q(2) - pi/2)*sin(q(3) + pi/2) - cos(q(2) - pi/2)*cos(q(3) + pi/2),            0,                                95 - 100*cos(q(2) - pi/2)*sin(q(3) + pi/2) - 100*cos(q(3) + pi/2)*sin(q(2) - pi/2) - 100*sin(q(2) - pi/2);
                                                                                                                    0,                                                                                                       0,            0,                                                                                                                                                  1];
            disp(T);
            
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
        



        
        
    end
end
