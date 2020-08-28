# Team Code

Code can be found in the "Gators" folder

## RobotHardware
This is the class that establishes the hardware for the robot.
For every motor on the robot, there is a class instance of either a DcMotor or a Servo.

```java
public DcMotor frontLeft;      // front left wheel
public DcMotor frontRight; // front right wheel
public DcMotor backLeft;       // back left wheel
public DcMotor backRight;  // back right wheeel

public DcMotor armRaisingMotor;     // motor to raise and lower the arm
public DcMotor armBaseMotor;        // motor to rotate the arm
public DcMotor armExtensionMotor;   // motor to extend the arm

public Servo   leftClawServo;       // servo controlling left side of the claw
public Servo   rightClawServo;      // servo controlling right side of the claw
```

There are also some basic functions for the control of the robot, able to be called by the DefaultDrive class or the AutoOp class

```java
void rotateArm(double torque) {
    armBaseMotor.setPower(torque);
}

void raiseArmBase(double torque) {
    armRaisingMotor.setPower(torque);
}

void extendArm(double power) {
    armExtensionMotor.setPower(power);
}
```

## DefaultDrive
Control class for remote control of the robot

Current control setup:
### Gamepad 1 - Robot Movement
 * each of the joysticks controls the respective side of the robot. To go forward, press both joysticks forward. To turn right, push the left joystick up and pull back the right joystick.
 * "Strafing" is controlled by the bumpers (these are located on the front side of the controller above the triggers)
 * Right bumper -> strafe right
 * Left bumper  -> strage left
 
 ### Gamepad 2 - Arm Control
 If you play video games, then controlling the robot is going to come a lot easier to you. The way it is set up is "inverted", meaning that pulling back on the right trigger makes the arm go up, and pushing forward makes it go down.
 * Pull back on the right trigger to lift the arm, and vise versa
 * Pushing the left joystick firwards extends the arm, and pulling it back retracts it.
 * Press the right bumper to open the claw, and left bumper to close it
 * Moving the right joystick left and right will rotate the arm.

Note: A guide to programming control classes can be found [here](https://github.com/ftctechnh/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md)
