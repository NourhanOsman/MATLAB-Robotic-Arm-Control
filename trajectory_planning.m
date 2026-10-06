% decleration the relations between the joints, define all distances in
% metres
clear all
clear a

L0=0.051;
L1=0.0635;
L2=0.0785;
L3=0.0765;

a = arduino
Base = servo(a, 'D9')
Shoulder = servo(a, 'D6')
Elbow = servo(a, 'D11')

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
tform = trvec2tform([0 , 0, L0]); % tform relates base to joint 1. the function (vector to 4x4 matrix)
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

robot = rigidBodyTree; % base axis

addBody(robot,body1,'base');


body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
tform2 = trvec2tform([0, 0, L1])*eul2tform([0,0,pi/2]);
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
tform3 = trvec2tform([L2, 0, 0])*eul2tform([0,0,pi]);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2



bodyEndEffector = rigidBody('endeffector');

tform4 = trvec2tform([L3, 0, 0]); % User defined

setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');

y=0.10;

t1= trvec2tform ([0.1 y 0.1]);
t2= trvec2tform ([0.1 y 0.15]);
t3= trvec2tform ([-0.1 y 0.15]);
t4= trvec2tform ([-0.1 y 0.1]);


n=16;


tInterval = [0 1.3];
tvec = 0:0.1:1.5 ;


[tfInterp1, v1, a1] = transformtraj(t1,t2,tInterval,tvec);
[tfInterp2, v1, a1] = transformtraj(t2,t3,tInterval,tvec);
[tfInterp3, v1, a1] = transformtraj(t3,t4,tInterval,tvec);
[tfInterp4, v1, a1] = transformtraj(t4,t1,tInterval,tvec);
initialguess = robot.homeConfiguration;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for i=1:1:n

       ik = inverseKinematics('RigidBodyTree',robot); %inverse kine operator
    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;
    [configSol1,solInfo] = ik('endeffector',tfInterp1(:,:,i),weights,initialguess);

 %to get intialguess close to the true position

    theta=(configSol1(1).JointPosition*180)/pi;
    alpha=(configSol1(2).JointPosition*180)/pi;
    gamma=(configSol1(3).JointPosition*180)/pi;

    if (alpha < 0 && gamma >0)
        alpha = abs (alpha );
        gamma = gamma + alpha;
    end

    if (gamma < 0 && alpha >0)
        gamma = abs (gamma );
        alpha = alpha + gamma;
    end
    
    if  (gamma <0 && alpha <0)
       if (abs(gamma) > abs(alpha))
           gamma=abs (gamma) 
           alpha=alpha+gamma
       end
       if (abs(alpha) > abs(gamma))
           alpha=abs (alpha) 
           gamma=gamma+alpha
       end
    end
    writePosition(Base,abs(theta/180));
    writePosition(Shoulder,(alpha/180));
    writePosition(Elbow,(gamma/180));
    
 end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for i=1:1:n

       ik = inverseKinematics('RigidBodyTree',robot); %inverse kine operator
    weights = [0 0 0 1 1 1];
    
    
    
    
    
    
    [configSol2,solInfo] = ik('endeffector',tfInterp2(:,:,i),weights,initialguess);
    
     %to get intialguess close to the true position
    initialguess(1).JointPosition= configSol2(1).JointPosition;
    initialguess(2).JointPosition= configSol2(2).JointPosition;
    initialguess(3).JointPosition= configSol2(3).JointPosition;

    theta=(configSol2(1).JointPosition*180)/pi;
    alpha=(configSol2(2).JointPosition*180)/pi;
    gamma=(configSol2(3).JointPosition*180)/pi;

    if (alpha < 0 && gamma >0)
        alpha = abs (alpha );
        gamma = gamma + alpha;
    end

    if (gamma < 0 && alpha >0)
        gamma = abs (gamma );
        alpha = alpha + gamma;
    end

    if  (gamma <0 && alpha <0)
       if (abs(gamma) > abs(alpha))
           gamma=abs (gamma) 
           alpha=alpha+gamma
       end
       if (abs(alpha) > abs(gamma))
           alpha=abs (alpha) 
           gamma=gamma+alpha
       end
    end
    writePosition(Base,abs(theta/180));
    writePosition(Shoulder,(alpha/180));
    writePosition(Elbow,(gamma/180));
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for i=1:1:n
    
       ik = inverseKinematics('RigidBodyTree',robot); %inverse kine operator
    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;
    [configSol3,solInfo] = ik('endeffector',tfInterp3(:,:,i),weights,initialguess);

    %to get intialguess close to the true position
    initialguess(1).JointPosition= configSol3(1).JointPosition;
    initialguess(2).JointPosition= configSol3(2).JointPosition;
    initialguess(3).JointPosition= configSol3(3).JointPosition;
    
    
    theta=(configSol3(1).JointPosition*180)/pi;
    alpha=(configSol3(2).JointPosition*180)/pi;
    gamma=(configSol3(3).JointPosition*180)/pi;

    if (alpha < 0 && gamma >0)
        alpha = abs (alpha );
        gamma = gamma + alpha;
    end

    if (gamma < 0 && alpha >0)
        gamma = abs (gamma );
        alpha = alpha + gamma;
    end
 
    if  (gamma <0 && alpha <0)
       if (abs(gamma) > abs(alpha))
           gamma=abs (gamma) 
           alpha=alpha+gamma
       end
       if (abs(alpha) > abs(gamma))
           alpha=abs (alpha) 
           gamma=gamma+alpha
       end
    end

    writePosition(Base,abs(theta/180));
    writePosition(Shoulder,(alpha/180));
    writePosition(Elbow,(gamma/180));
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for i=1:1:n
     
       ik = inverseKinematics('RigidBodyTree',robot); %inverse kine operator
    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;
    [configSol4,solInfo] = ik('endeffector',tfInterp4(:,:,i),weights,initialguess);

    %to get intialguess close to the true position
    initialguess(1).JointPosition= configSol4(1).JointPosition;
    initialguess(2).JointPosition= configSol4(2).JointPosition;
    initialguess(3).JointPosition= configSol4(3).JointPosition;
    
    theta=(configSol4(1).JointPosition*180)/pi
    alpha=(configSol4(2).JointPosition*180)/pi
    gamma=(configSol4(3).JointPosition*180)/pi

    if (alpha < 0 && gamma >0)
        alpha = abs (alpha );
        gamma = gamma + alpha;
    end

    if (gamma < 0 && alpha >0)
        gamma = abs (gamma );
        alpha = alpha + gamma;
    end
    
    if  (gamma <0 && alpha <0)
       if (abs(gamma) > abs(alpha))
           gamma=abs (gamma) 
           alpha=alpha+gamma
       end
       if (abs(alpha) > abs(gamma))
           alpha=abs (alpha) 
           gamma=gamma+alpha
       end
    end

    writePosition(Base,abs(theta/180));
    writePosition(Shoulder,(alpha/180));
    writePosition(Elbow,(gamma/180));
 end
