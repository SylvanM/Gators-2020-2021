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

package org.firstinspires.ftc.teamcode.Gators.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware
{

    /* Constants and properties */

    // in radians per second
    public final double ANGULAR_SPEED = Math.PI * 3  / 4.7;
    public final double LINEAR_SPEED  = 1.75 / 3.43; // meters per second

    private final double[] weights = {
            1.0, 1.0,
            1.0, 1.0
    };

    private DcMotor[] motors = new DcMotor[4];

    /* local OpMode members. */
    private ElapsedTime period = new ElapsedTime();

    // MARK: Constructors
    // construct with just map and instructions and value for lift power
    public RobotHardware(HardwareMap ahwMap, double power) {
        setRobotProperties(ahwMap);
        liftPower = power;
    }

    public RobotHardware(HardwareMap ahwMap) {
        setRobotProperties(ahwMap);
    }

    /* Initialize standard Hardware interfaces */

    /**
     * Function that initializes the robot from a hardware map
     * @param hwMap Hardware map to use to initialize the hardware class
     */
    public void init(HardwareMap hwMap) {
        setRobotProperties(hwMap);
    }

    // MARK: Methods

    // This simply sets the robot's stuff given certain instructions
    // sorry I know that's the least helpful comment ever written
    private void setRobotProperties(HardwareMap map) {
        int k;
        String motorName;

        for ( k = 0; k < 4; ++k ) {
            // get motor name
            motorName =  (k < 2)      ? "front_" : "back_";
            motorName += (k % 2 == 0) ? "left" : "right";

            motors[k] = map.get(DcMotor.class, motorName);
            if (motors[k] == null) continue;
            motors[k].setDirection( (k % 2 == 0) ? (DcMotor.Direction.REVERSE) : (DcMotor.Direction.FORWARD) );
        }

    }

    private void setMotor(int motorIndex, double power, boolean weighted) {
        if ( motors[motorIndex] == null )
            return;

        motors[motorIndex].setPower(power * ( (weighted) ? (weights[motorIndex]) : (1)));
    }

    /**
     * Function that will move the robot
     * @param angle to calculate angle and magnitude
     * @param distance to set the distance to travel at max speed
     */
    public void moveDistance(double angle, double distance) {
        double time = distance * 1000 / LINEAR_SPEED;
        double startTime = period.milliseconds();
        double deltaTime;
        do {
            deltaTime = period.milliseconds() - startTime;
            moveBot(angle - 135, 1, 0);
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

