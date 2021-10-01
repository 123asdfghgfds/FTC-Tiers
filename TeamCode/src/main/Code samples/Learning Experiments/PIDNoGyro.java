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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="PID", group="Linear Opmode")

public class PIDNoGyro extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members
    private DcMotor     leftDrive1 = null;
    private DcMotor     rightDrive1 = null;
    private DcMotor     leftDrive2 = null;
    private DcMotor     rightDrive2 = null;

    //Encoder setup
    public static double    kp = 0.003; // kp=1/(704.86*constant) = constant = 0.7; //0.04
    static final double     COUNTS_PER_MOTOR_REV = 746.6;
    static final double     WHEEL_DIAMETER_INCHES = 4;
    static final double     Counts_PER_INCH = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_INCHES * 3.1415);

    //PID
    public static PIDCoefficients shooterPID = new PIDCoefficients(0.0047332348820,0,0);
    ElapsedTime PIDTimer = new ElapsedTime();
    double integral = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Drive
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

        //Pregame setup
        resetEncoders();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

    }

    private void PID_Drive(double distance){
        resetEncoders();

        double error = 0;
        double lastError = 0;
        double velocity;
        int target = (int)(distance * Counts_PER_INCH);

        error = target - getCurrentPosition();
        double t1=0;
        PIDTimer.reset();

        while(opModeIsActive()){

            error = target - getCurrentPosition();

            double changInError = lastError - error;
            double t2 =PIDTimer.time();
            integral += changInError*PIDTimer.time();
            double derivative = changInError;///(t2 - t1);

            double P = shooterPID.p * error;
            double I = shooterPID.i * integral;
            double D = shooterPID.d * derivative;

            velocity = P+I+D;
            velocity = Range.clip((velocity), -1.0, 1.0);

            motorSet(velocity,velocity);

            lastError = error;
            t1 = t2;
        }
        motorSet(0,0);
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

    //Shortcuts
    private void motorSet(double left, double right) {
        leftDrive1.setPower(left);
        leftDrive2.setPower(left);
        rightDrive1.setPower(right);
        rightDrive2.setPower(right);
    }
}
