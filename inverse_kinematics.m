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

while 1
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
tform3 = trvec2tform([L2, 0, 0]);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2

bodyEndEffector = rigidBody('endeffector');

tform4 = trvec2tform([L3, 0, 0]); % User defined

setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');


x= -0.0615;
y= -0.0224;
z= 0.2544; 

xyz=[1 0 0 x;
     0 1 0 y;
     0 0 1 z;
     0 0 0 1];

ik = inverseKinematics('RigidBodyTree',robot); %inverse kine operator
weights = [0 0 0 1 1 1];
initialguess = robot.homeConfiguration;
[configSol,solInfo] = ik('endeffector',xyz,weights,initialguess);
theta=(configSol(1).JointPosition*180)/pi
alpha=(configSol(2).JointPosition*180)/pi
gamma=(configSol(3).JointPosition*180)/pi

writePosition(Base,abs(theta/180));
writePosition(Shoulder,abs(alpha/180));
writePosition(Elbow,abs(gamma/180));

end
