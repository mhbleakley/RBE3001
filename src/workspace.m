[x,y,z] = sphere;               % Makes a 21-by-21 point sphere
x = x(11:end,:);                % Keep top 11 x points
y = y(11:end,:);                % Keep top 11 y points
z = z(11:end,:);                % Keep top 11 z points
r = 140;                        % A radius value
hs = surf(r.*x,r.*y,(r+55).*z);   % Plot the surface
direction = [0 1 0];            % Specify Direction
axis equal;                     % Make the scaling on the x, y, and z axes equal
Ax = get(gca);                  % Axes Handle

title("Serial Arm Workspace");
xlabel('X Axis') ;
ylabel('Y Axis'); 
zlabel ('Z Axis');
