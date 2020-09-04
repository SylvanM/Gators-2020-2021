/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



// TEST CHANGE

package org.firstinspires.ftc.teamcode.Gators.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardware
{

    /*
     * Telemetry object so the hardware can relay telemetry
     */
    Telemetry telemetry;

    /* Constants and properties */

    public static final double ANGLE_OFFSET = 135;

    // in radians per second
    public final double ANGULAR_SPEED = Math.PI * 3  / 4.7;
    public final double LINEAR_SPEED  = 1.75 / 3.43; // meters per second

    private final double[] weights = {
            1.0, 1.0,
            1.0, 1.0
    };

    /* Hardware Components */

    /*
     * The motors on the robot that make it move (aka the wheels)
     */
    private DcMotor[] motors = new DcMotor[4];

    /*
     * Onboard sensors
     */

    // Inertial Measurement Unit
    private BNO055IMU imu;

    /* local OpMode members. */
    private ElapsedTime period = new ElapsedTime();

    // MARK: Constructors
    // construct with just map and instructions and value for lift power
    public RobotHardware(HardwareMap ahwMap, double power) {
        setRobotProperties(ahwMap);
    }

    public RobotHardware(HardwareMap ahwMap, Telemetry telem) {
        telemetry = telem;
        setRobotProperties(ahwMap);
    }

    /* Initialize standard Hardware interfaces */

    /**
     * Function that initializes the robot from a hardware map
     * @param hwMap Hardware map to use to initialize the hardware class
     */
    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        setRobotProperties(hwMap);
    }

    // MARK: Methods

    // This simply sets the robot's stuff given certain instructions
    // sorry I know that's the least helpful comment ever written
    private void setRobotProperties(HardwareMap map) {
        int k;
        String motorName;

        telemetry.addLine("Initializing robot hardware");

        telemetry.addLine("Initializing wheel motors...");
        // initialize the wheel motors
        for ( k = 0; k < 4; ++k ) {
            // get motor name
            motorName =  (k < 2)      ? "front_" : "back_";
            motorName += (k % 2 == 0) ? "left" : "right";

            motors[k] = map.get(DcMotor.class, motorName);

            if (motors[k] == null) {
                telemetry.addLine("Could not find motor '" + motorName + "', skipping.");
                continue;
            }

            motors[k].setDirection( (k % 2 == 0) ? (DcMotor.Direction.REVERSE) : (DcMotor.Direction.FORWARD) );
            motors[k].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[k].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addLine("Initialized '" + motorName + "'");
        }

        // set up the IMU
        telemetry.addLine("Initializing IMU...");
        imu = map.get(BNO055IMU.class, "imu");

        if ( imu != null ) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            imu.initialize(parameters);
            telemetry.addLine("Initialized IMU.");
        } else {
            telemetry.addLine("Could not find IMU.");
        }

    }

    private void setMotor(int motorIndex, double power, boolean weighted) {
        if ( motors[motorIndex] == null )
            return;

        motors[motorIndex].setPower(power * ( (weighted) ? (weights[motorIndex]) : (1)));
    }

    /**
     * Moves the robot a certain distance in a certain direction
     * @param angle to calculate angle and magnitude
     * @param distance to set the distance to travel at max speed
     */
    public void moveDistance(double angle, double distance) {
        double time = distance * 1000 / LINEAR_SPEED;
        double startTime = period.milliseconds();
        double deltaTime;
        do {
            deltaTime = period.milliseconds() - startTime;
            moveBot(angle, 1, 0);
        } while (deltaTime <= time);
    }

    /**
     * Function that will move the robot
     */
    public void rotateTheta(double displacement) {
        double time = (Math.abs(displacement) * 1000) / ANGULAR_SPEED;
        double startTime = period.milliseconds();
        double deltaTime;
        do {
            deltaTime = period.milliseconds() - startTime;
            moveBot(0,0, displacement);
        } while (deltaTime <= time);
    }

    /**
     * Function that will move the robot
     * @param angle to calculate angle and magnitude
     * @param rotarySpeed to set the rotary speed for the wheels
     */
    public void moveBot(double angle, double speed, double rotarySpeed) {

        setMotor(0, speed * Math.sin(Math.toRadians(angle)) + rotarySpeed, true);
        setMotor(1, speed * Math.cos(Math.toRadians(angle)) - rotarySpeed, true);
        setMotor(2, speed * Math.cos(Math.toRadians(angle)) + rotarySpeed, true);
        setMotor(3, speed * Math.sin(Math.toRadians(angle)) - rotarySpeed, true);

    }
}

