%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos;
        
        %drone angles
        theta; 
        
        %drone velocities
        xdot;
        
        %drone angular velocities
        omega;
        
        %drone rotation matrix
        R
        
        %Controller parameters
        integral_err_z_pos
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0;0;0];
                
                obj.theta = zeros(3, 1); 
                
                obj.xdot = zeros(3, 1);
                
                obj.omega = zeros(3, 1);
                
                obj.integral_err_z_pos = 0;
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            if(obj.drone_follow)
               axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!
        function obj = change_pos_and_orientation(obj)
            t = obj.time;
            
            %vary position
            pos_mat = [2.6*sin(t/2), 2.1*cos(t/5),(2.4*sin(t/3)*sin(t/3))] + obj.pos_offset;
            
            %vary orientation
            pitch = 0.3*sin(t*15.2);
            roll = 0.1*cos(t*33.1 + 0.5);
            yaw = 2.*pi*sin(t);
            rot_mat = eul2rotm([yaw, roll, pitch]);
            
            %update position and rotation matrix of drone
            obj.pos = pos_mat;
            obj.R = rot_mat;
        end
        
        function obj = first_move(obj)
            I_xx = 0.21;
            I_yy = 0.21;
            I_zz = 0.1;
            m = 2;
            kd = 0.2;
            g = [0; 0; -9.8];
            
            R_mat = [cos(obj.theta(1))*cos(obj.theta(3))-cos(obj.theta(2))*sin(obj.theta(1))*sin(obj.theta(3))...
                -cos(obj.theta(3))*sin(obj.theta(1))-cos(obj.theta(1))*cos(obj.theta(2))*sin(obj.theta(3))...
                sin(obj.theta(2))*sin(obj.theta(3));...
                cos(obj.theta(2))*cos(obj.theta(3))*sin(obj.theta(1))+cos(obj.theta(1))*sin(obj.theta(3))...
                cos(obj.theta(1))*cos(obj.theta(2))*cos(obj.theta(3))-sin(obj.theta(1))*sin(obj.theta(3))...
                -cos(obj.theta(3))*sin(obj.theta(2));...
                sin(obj.theta(1))*sin(obj.theta(2))...
                cos(obj.theta(1))*sin(obj.theta(2))...
                cos(obj.theta(2))];
            
            I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];
            
            % Control the Thrust and Torque as Inputs
            Kp1 = 15.0;%20.0;
            Kd1 = 6.0;
            Ki1 = 6.0; %6.0
            
            Kp2 = 3;
            Kd2 = 3;
           
            desired_z_pos = 2.5;
            desired_z_dot = 0;
            desired_theta = [0; 0; 0];
            desired_omega = [0; 0; 0];
            
            integral_z_pos = obj.integral_err_z_pos + (obj.pos(3) - desired_z_pos) * obj.time_interval;
            T_B_z = -Kp1*(obj.pos(3) - desired_z_pos) - Kd1*(obj.xdot(3) - desired_z_dot) - Ki1*integral_z_pos;
            obj.integral_err_z_pos = integral_z_pos;
            T_B = [0; 0; T_B_z];
            
            torques = -Kp2*(obj.theta - desired_theta) - Kd2*(obj.omega - desired_omega); 
            %R_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]); % ** Unsure
            a = g + 1 / m * (R_mat*(T_B)) - kd .* obj.xdot;
            
            omegadot = inv(I) * (torques - cross(obj.omega, I * obj.omega));       
           
            obj.omega = obj.omega + obj.time_interval * omegadot;
            thetadot = inv([1 0 -sin(obj.theta(2)); 0 cos(obj.theta(1)) cos(obj.theta(2))*sin(obj.theta(1)); 0 -sin(obj.theta(1)) cos(obj.theta(2))*cos(obj.theta(1))]) * obj.omega;
            obj.theta = obj.theta + obj.time_interval * thetadot;
            obj.xdot = obj.xdot + obj.time_interval * a;
            obj.pos = obj.pos + obj.time_interval * obj.xdot;
            
            disp("Position: ")
            disp(obj.pos);
%             disp("Linear velocities: ")
%             disp(obj.xdot);
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
        end
        
        function obj = second_move(obj)
            obj.pos = [0; 0; 2.5];
            
            I_xx = 0.21;
            I_yy = 0.21;
            I_zz = 0.1;
            m = 2;
            kd = 0.2;
            g = [0; 0; -9.8];
            
            R_mat = [cos(obj.theta(1))*cos(obj.theta(3))-cos(obj.theta(2))*sin(obj.theta(1))*sin(obj.theta(3))...
                -cos(obj.theta(3))*sin(obj.theta(1))-cos(obj.theta(1))*cos(obj.theta(2))*sin(obj.theta(3))...
                sin(obj.theta(2))*sin(obj.theta(3));...
                cos(obj.theta(2))*cos(obj.theta(3))*sin(obj.theta(1))+cos(obj.theta(1))*sin(obj.theta(3))...
                cos(obj.theta(1))*cos(obj.theta(2))*cos(obj.theta(3))-sin(obj.theta(1))*sin(obj.theta(3))...
                -cos(obj.theta(3))*sin(obj.theta(2));...
                sin(obj.theta(1))*sin(obj.theta(2))...
                cos(obj.theta(1))*sin(obj.theta(2))...
                cos(obj.theta(2))];
            
            I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];
            
            % Control the Thrust and Torque as Inputs
            Kp1 = 15.0;%20.0;
            Kd1 = 6.0;
            Ki1 = 6.0; %6.0
            
            Kp2 = 3;
            Kd2 = 3;
           
            desired_z_pos = 2.5;
            desired_z_dot = 0;
            desired_theta = [0; 0; 0];
            desired_omega = [2; 2; 0];
            
            integral_z_pos = obj.integral_err_z_pos + (obj.pos(3) - desired_z_pos) * obj.time_interval;
            T_B_z = -Kp1*(obj.pos(3) - desired_z_pos) - Kd1*(obj.xdot(3) - desired_z_dot) - Ki1*integral_z_pos;
            obj.integral_err_z_pos = integral_z_pos;
            T_B = [0; 0; T_B_z];
            
            torques = -Kp2*(obj.theta - desired_theta) - Kd2*(obj.omega - desired_omega); 
            %R_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]); % ** Unsure
            a = g + 1 / m * (R_mat*(T_B)) - kd .* obj.xdot;
            
            omegadot = inv(I) * (torques - cross(obj.omega, I * obj.omega));       
           
            obj.omega = obj.omega + obj.time_interval * omegadot;
            thetadot = inv([1 0 -sin(obj.theta(2)); 0 cos(obj.theta(1)) cos(obj.theta(2))*sin(obj.theta(1)); 0 -sin(obj.theta(1)) cos(obj.theta(2))*cos(obj.theta(1))]) * obj.omega;
            obj.theta = obj.theta + obj.time_interval * thetadot;
            obj.xdot = obj.xdot + obj.time_interval * a;
            obj.pos = obj.pos + obj.time_interval * obj.xdot;
            
            disp("Position: ")
            disp(obj.pos);
%             disp("Linear velocities: ")
%             disp(obj.xdot);
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
        end
        
      
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
            %obj = change_pos_and_orientation(obj);
            obj = second_move(obj);
            
            %draw drone on figure
            draw(obj);
        end
    end
end
