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
tform3 = trvec2tform([L2, 0, 0]);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2



bodyEndEffector = rigidBody('endeffector');

tform4 = trvec2tform([L3, 0, 0]); % User defined

setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');

angle1= 0;      %giving angle base 
writePosition(Base,angle1/180);

angle2= 0;      %giving angle shoulder 
writePosition(Shoulder,angle2/180);

angle3= 0;      %giving angle elbow 
writePosition(Elbow,angle3/180);

config = randomConfiguration(robot);
config(1).JointPosition= (angle1*pi)/180; %servo1
config(2).JointPosition= (angle2*pi)/180;%servo2
config(3).JointPosition= -(angle3*pi)/180; %servo3 opposite side of the other servo


forward_trans = getTransform(robot,config,'endeffector','base'); %forward transformation. takes theta struct and outputs homog. matrix

x=forward_trans(1,4)
y=forward_trans(2,4)
z=forward_trans(3,4)



 
