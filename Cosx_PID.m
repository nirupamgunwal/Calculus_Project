vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('Connected to remote API server');
         
   
        
        %handles
        [returnCode,Left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
        [returnCode,Right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)
       
        [returnCode,Poineer_p3dx]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking)
        %code
      
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Poineer_p3dx,-1,vrep.simx_opmode_blocking)
        disp(position);
        
        [returnCode,orientation]=vrep.simxGetObjectPosition(clientID,Poineer_p3dx,-1,vrep.simx_opmode_blocking)
        disp(orientation);
        
       
       %code starts here
        x_arr = linspace(position(1),6,50)
       for i = 1:50
          y_arr(i) = cos(x_arr(i));  
        end


        integral = 0;
        e2 = 0;
      
        for i = 1:50
        x0 = x_arr(i);
        y0 = y_arr(i);
        while(~(position(1)>x0-0.1 && position(1)<x0 + 0.1 && position(2)<y0 + 0.1 && position(2) >y0 -0.1))
        
       
            
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Poineer_p3dx,-1,vrep.simx_opmode_blocking)
        disp(position);
        
        [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,Poineer_p3dx,-1,vrep.simx_opmode_blocking)
        
        disp(eulerAngles);
        
        v = 20;
        k = 200;
        k2 = 10;
        k3 = 1;
        
        r = 0.09;
        theta = atan((y0 - position(2))/(x0 - position(1)));
        l = .5;
        
        
        error = theta - eulerAngles(3);
        
        e1 = atan2(sin(error),cos(error)); 
        lasterror = e2;
        derivative= e1 -lasterror;
        disp(derivative);
        e2 = e1
        integral=integral + e1;
        
        w = k*e1 + k2*derivative + k3*integral;
       
       
        vr = (2*v + w*l )/ 2*r;
        vl = (2*v - w*l) / 2*r;
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Left_Motor,vl,vrep.simx_opmode_blocking)
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Right_Motor,vr,vrep.simx_opmode_blocking)
      
     
        end
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Left_Motor,0,vrep.simx_opmode_blocking)
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Right_Motor,0,vrep.simx_opmode_blocking)
        pause(0.1);
        end
        %pause(10)
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Left_Motor,0,vrep.simx_opmode_blocking)
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Right_Motor,0,vrep.simx_opmode_blocking)
        pause(10);
        vrep.simxFinish(-1); 
  
        end

      


vrep.delete();