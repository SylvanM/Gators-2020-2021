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

package org.firstinspires.ftc.teamcode.Gators.TeleOp;

import org.firstinspires.ftc.teamcode.Gators.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Gators.Hardware.RobotHardware;

import java.util.concurrent.TimeUnit;

// GAMEPAD 1 controls MOTION of robot
// GAMEPAD 2 controls ACTIONS of robot (moving the arm, stuff like that)

@TeleOp(name="Human control", group="Omnidirectional Control")
public class HumanControl extends OpMode {

    /* Constants */



    /* Declare OpMode members. */
    private RobotHardware robot;

    int height;
    int power = 10;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // hey
        // Code runs on init-button pressed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new RobotHardware(hardwareMap, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("In loop");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.clear();

        // Run on start
        telemetry.addLine("Driver in control, inputs now will have effect on robot.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // might have to remove this
        telemetry.clear();

        /*
         * WHEEL MOVEMENT
         */

        // math for calculating standard control

        double x = (double) gamepad1.left_stick_x;
        double y = (double) gamepad1.left_stick_y;

        double angle       = getAngle(x, y);
        double linearSpeed = getMagnitude(x, y);
        double rotarySpeed = (double) gamepad1.right_stick_x;

        robot.moveBot(angle, linearSpeed, rotarySpeed);

        /*
         * MOVEMENT TELEMETRY
         */

        telemetry.addData("Direction:", angle);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop

    }

    // methods

    // Returns the angle of the vector with components x and y
    private double getAngle(double x, double y) {
        double angle = Math.toDegrees(Math.atan2(y, x));
        telemetry.addData("x:", x);
        telemetry.addData("y:", y);

        if (angle < 0) {
            angle += 360;
        } else if (angle > 360) {
            angle -= 360;
        }

        return angle + RobotHardware.ANGLE_OFFSET;
    }

    // Returns the magnitude of a vector with components a and b
    private double getMagnitude(double a, double b) {
        float exponent = 2;
        float aSquared = (float) Math.pow(a, exponent);
        float bSquared = (float) Math.pow(b, exponent);
        double sum = (double)aSquared + (double)bSquared;

        return Math.sqrt(sum);

    }


}