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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumTeleop", group="Linear Opmode")

public class MecanumTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Train
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    //Gyro Setup
    BNO055IMU imu;
    Orientation angles;

    //Other setup
    boolean pressed;
    public static double kp = 0.003;

    @Override
    public void runOpMode() {

        //Drive Train
        leftDrive1 = hardwareMap.get(DcMotor.class, "left1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right2");

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Drive Train
            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            if (gamepad1.left_bumper) {
                StrafeLeft(angles.firstAngle);
                pressed = true;
            }

            else if (gamepad1.right_bumper) {
                StrafeRight(angles.firstAngle);
                pressed = true;
            }

            else {
                leftDrive1.setPower(leftPower);
                rightDrive1.setPower(rightPower);
                leftDrive2.setPower(leftPower);
                rightDrive2.setPower(rightPower);

                pressed = false;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
    }

    //Strafing
    private void StrafeLeft(double angle) {

        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        if (opModeIsActive()) {

            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && pressed) {

                if (!gamepad1.left_bumper)
                    pressed = false;
            }
            error = getErrorAngle(angle);
            steer = getSteer(error, kp);

            left1Speed = 0.9 - steer;
            left2Speed = 0.9 + steer;

            right1Speed = 0.9 - steer;
            right2Speed = 0.9 + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));
            max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

            if (max1 > 1.0) {
                left1Speed /= max1;
                right1Speed /= max1;

            }
            if (max2 > 1.0) {
                left2Speed /= max2;
                right2Speed /= max2;
            }

            leftDrive1.setPower(left1Speed);
            leftDrive2.setPower(left2Speed);
            rightDrive1.setPower(right1Speed);
            rightDrive2.setPower(right2Speed);
        }
    }
    private void StrafeRight(double angle) {
        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        if (opModeIsActive()) {

            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && pressed) {

                if (!gamepad1.left_bumper)
                    pressed = false;
            }
            error = getErrorAngle(angle);
            steer = getSteer(error, kp);

            left1Speed = 0.9 + steer;
            left2Speed = 0.9 - steer;

            right1Speed = 0.9 + steer;
            right2Speed = 0.9 - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));
            max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

            if (max1 > 1.0) {
                left1Speed /= max1;
                right1Speed /= max1;

            }
            if (max2 > 1.0) {
                left2Speed /= max2;
                right2Speed /= max2;
            }

            leftDrive1.setPower(left1Speed);
            leftDrive2.setPower(left2Speed);
            rightDrive1.setPower(right1Speed);
            rightDrive2.setPower(right2Speed);
        }
    }

    //Gyro functions
    private double getErrorAngle(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

