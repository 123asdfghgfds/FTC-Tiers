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

//Core

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="TierFourAuto", group="Linear Opmode")

public class ProportionalDrive extends LinearOpMode {

    // Declare OpMode members
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    //Encoder setup
    public static double    kp = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7; //0.04
    static final double     COUNTS_PER_MOTOR_REV = 746.6;
    static final double     WHEEL_DIAMETER = 10.16;
    static final double     Counts_PER_CM = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER * 3.1415)/1.65;

    //Gyro setup
    BNO055IMU imu;
    Orientation angles;
    Orientation last_angle = new Orientation();
    double angle;

    @Override
    public void runOpMode() throws InterruptedException {

        //Gyro initialise
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Mapping
        leftDrive1  = hardwareMap.get(DcMotor.class, "left1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right2");

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        
        //Pregame setup
        resetEncoders();
        resetGyro();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //Movement
        PGyroDrive(0.5,70,0);
    }

    private void PGyroDrive(double power, double distanceCM, double angle) {

        //Declare Variables
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  adjustment;
        double  leftSpeed;
        double  rightSpeed;
        double  velocity;
        double  gain;

        adjustment = 0.03;

        //Reduce Error
        moveCounts = (int)(distanceCM * Counts_PER_CM);
        newLeftTarget = leftDrive1.getCurrentPosition() + moveCounts;
        newRightTarget = rightDrive1.getCurrentPosition() + moveCounts;

        leftDrive1.setTargetPosition(newLeftTarget);
        leftDrive2.setTargetPosition(newLeftTarget);
        rightDrive1.setTargetPosition(newRightTarget);
        rightDrive2.setTargetPosition(newRightTarget);

        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Range.clip(Math.abs(power), 0.0, 1.0); //Make sure power doesn't go over range

        while (opModeIsActive() && leftDrive1.isBusy() && leftDrive2.isBusy() && rightDrive1.isBusy() && rightDrive2.isBusy()) {

            //Proportional Speed Modifier
            gain = calcGain(moveCounts);

            if (distanceCM < 0) {
                gain *= -1.0;
            }
            velocity = power * gain;
            velocity = Range.clip(Math.abs(velocity), 0.0, 1.0);

            //Angle Controller
            if (getErrorAngle(angle) < 0) {
                leftSpeed = velocity * (1 + adjustment);
                rightSpeed = velocity * (1 - adjustment);
            }
            else if (getErrorAngle(angle) > 0) {
                leftSpeed = velocity * (1 - adjustment);
                rightSpeed = velocity * (1 + adjustment);
            }
            else {
                leftSpeed = velocity;
                rightSpeed = velocity;
            }

            //Make sure motor power does not go over 1
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            motorSet(leftSpeed, rightSpeed);
        }
        motorStop();
    }

    //Gyro Functions
    private void resetGyro(){
        last_angle  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angle = 0;
    }
    private double getErrorAngle(double targetAngle) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //Encoder Functions
    private void resetEncoders(){
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private double getCurrentPosition(){
        return Math.abs(leftDrive1.getCurrentPosition() + leftDrive2.getCurrentPosition()
                + rightDrive1.getCurrentPosition() + rightDrive2.getCurrentPosition())/4;
    }
    private double getErrorDistance(double Target){
        return Target - getCurrentPosition();
    }
    private double calcGain(double Target) {
        return Range.clip(getErrorDistance(Target) * kp, -1, 1);
    }

    //Functions that shorten code
    private void motorSet(double left, double right) {
        leftDrive1.setPower(left);
        leftDrive2.setPower(left);
        rightDrive1.setPower(right);
        rightDrive2.setPower(right);
    }
    private void motorStop() {
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorSet(0,0);
        resetEncoders();
    }
}
