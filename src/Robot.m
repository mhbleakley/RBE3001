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
            disp(packet);
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
            pause(0.5);
            self.measured_js(1,1);
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
        
        
        
    end
end
