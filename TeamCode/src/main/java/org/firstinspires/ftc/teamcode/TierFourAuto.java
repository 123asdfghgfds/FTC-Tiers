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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="TierFourAuto", group="Linear Opmode")

public class TierFourAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members
    private DcMotor     leftDrive1 = null;
    private DcMotor     rightDrive1 = null;
    private DcMotor     leftDrive2 = null;
    private DcMotor     rightDrive2 = null;
    private DcMotorEx   shootermotor = null;
    private DcMotor     feedermotor = null;

    //Encoder setup
    public static double    kp = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7; //0.04
    public static double    kp2 = 0.004;

    static final double     COUNTS_PER_MOTOR_REV = 746.6;
    static final double     WHEEL_DIAMETER = 10.16;
    static final double     Counts_PER_CM = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER * 3.1415)/1.65;
    static final double     Counts_PER_CM_STRAFE = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER * 3.1415) * 0.7874015748;

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

        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Motors
        //shootermotor = hardwareMap.get(DcMotorEx.class, "shooter");
        //feedermotor = hardwareMap.get(DcMotor.class, "feeder");

        //Pregame setup
        resetEncoders();
        resetGyro();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //Movement
        //PGyroDrive(0.3, -165, 0);
        gyroStrafe(0.2, 100, 0);
        gyroStrafe(0.2, -100, 0);
        //gyroStrafe(0.8,150,0);
        //sleep(2000);
        //gyroStrafe(0.8,-150,0);

        //AutoShoot();
    }

    private void gyroStrafe(double speed, double distance, double angle) {

        int newLeft1Target;
        int newRight1Target;
        int newLeft2Target;
        int newRight2Target;
        int moveCounts;
        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;
        int l1,l2,r1,r2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * Counts_PER_CM_STRAFE);

            newLeft1Target = leftDrive1.getCurrentPosition() - moveCounts;
            newLeft2Target = leftDrive2.getCurrentPosition() + moveCounts;

            newRight1Target = rightDrive1.getCurrentPosition() + moveCounts;
            newRight2Target = rightDrive2.getCurrentPosition() - moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeft1Target);
            leftDrive2.setTargetPosition(newLeft2Target);

            rightDrive1.setTargetPosition(newRight1Target);
            rightDrive2.setTargetPosition(newRight2Target);

            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorSet(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive1.isBusy() && rightDrive1.isBusy()) && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getErrorAngle(angle);
                steer = getSteer(error, kp2);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                //leftSpeed = speed - steer;
                //rightSpeed = speed + steer;

                left1Speed = speed + steer;
                left2Speed = speed - steer;

                right1Speed = speed + steer;
                right2Speed = speed - steer;

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

                telemetry.update();

            }

            // Stop all motion;
           motorStop();
        }
    }
    private void PGyroDrive(double power, double distanceCM, double angle) {

        //Declare Variables
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  velocity;
        double  gain;

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
            error = getErrorAngle(angle);
            steer = getSteer(error, kp);
            gain = calcGain(moveCounts);

            if (distanceCM < 0) {
                gain *= -1.0;
            }
            velocity = power * gain;
            velocity = Range.clip(Math.abs(velocity), 0.0, 1.0);

            //Angle Controller
            leftSpeed = velocity - steer;
            rightSpeed = velocity + steer;

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
    private void AutoShoot() {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3) {
            shootermotor.setVelocity(-1500);
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1500);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            shootermotor.setVelocity(-1500);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1500);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            shootermotor.setVelocity(-1500);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1500);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            feedermotor.setPower(0);
        }
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
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
