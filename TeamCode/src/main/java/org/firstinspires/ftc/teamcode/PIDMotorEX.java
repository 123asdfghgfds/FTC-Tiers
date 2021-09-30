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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

//Gyro
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//PID
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="TierThreeAuto", group="Linear Opmode")

public class PIDMotorEX extends LinearOpMode {

    // Declare OpMode members
    private DcMotorEx left1 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right2 = null;

    //PID
    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(25,0.05,1.25,0);
    PIDFCoefficients pidOrig,currentPID;

    //Encoder setup
    static final double     COUNTS_PER_MOTOR_REV    =  704.86 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

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
        left1  = (DcMotorEx)hardwareMap.get(DcMotor.class, "left1");
        right1 = (DcMotorEx)hardwareMap.get(DcMotor.class, "right1");
        left2  = (DcMotorEx)hardwareMap.get(DcMotor.class, "left2");
        right2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "right2");

        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //PID Setup
        pidOrig = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        currentPID = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //Pregame setup
        resetEncoders();
        resetGyro();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //Movement
        PIDDriveEX(0.5,100,0);
    }
    private void PIDDriveEX(double power, double distanceCM, double angle) {

        //Declare Variables
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  adjustment;
        double  leftSpeed;
        double  rightSpeed;

        adjustment = 0.03;

        //Reduce Error
        moveCounts = (int)(distanceCM * COUNTS_PER_INCH);
        newLeftTarget = left1.getCurrentPosition() + moveCounts;
        newRightTarget = right1.getCurrentPosition() + moveCounts;

        left1.setTargetPosition(newLeftTarget);
        left2.setTargetPosition(newLeftTarget);
        right1.setTargetPosition(newRightTarget);
        right2.setTargetPosition(newRightTarget);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Range.clip(Math.abs(power), 0.0, 1.0); //Make sure power doesn't go over range

        while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy() && right2.isBusy()) {

            if (getErrorAngle(angle) < 0) {
                leftSpeed = power * (1 + adjustment);
                rightSpeed = power * (1 - adjustment);
            }
            else if (getErrorAngle(angle) > 0) {
                leftSpeed = power * (1 - adjustment);
                rightSpeed = power * (1 + adjustment);
            }
            else {
                leftSpeed = power;
                rightSpeed = power;
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

    //Functions that shorten code
    private void motorSet(double left, double right) {
        left1.setPower(left);
        left2.setPower(left);
        right1.setPower(right);
        right2.setPower(right);
    }
    private void motorStop() {
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorSet(0,0);
        resetEncoders();
    }
    private void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

}
