import pybullet as p;
import pybullet_data as pd;
p.connect(p.GUI);
p.setPhysicsEngineParameter(enableFileCaching=0);
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0);
p.setGravity(0,0,-30);

## set the floor
plane_shape = p.createCollisionShape(p.GEOM_PLANE);
floor = p.createMultiBody(plane_shape,plane_shape);

## run simulation
p.setRealTimeSimulation(1);

## load the robot
myrob = p.loadURDF("myrob.urdf");

## set the motor
mode = p.VELOCITY_CONTROL;
for i in range(p.getNumJoints(myrob)):
    joint = p.getJointInfo(myrob,i);
    joint_name = p.getJointInfo(myrob,i)[1].decode("utf-8");
    joint_id = p.getJointInfo(myrob,i)[0];

    if("wheel" in joint_name):
        p.setJointMotorControl2(myrob,joint_id,controlMode=mode,targetVelocity=10);