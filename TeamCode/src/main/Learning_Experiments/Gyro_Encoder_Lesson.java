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

package Learning_Experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Robot hardware
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Gyro
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GyroLesson", group="Linear Opmode")

public class Gyro_Encoder_Lesson extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    Orientation last_angle = new Orientation();
    double angle;

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    Servo claw = null;

    //Encoder setup
    static final double COUNTS_PER_MOTOR_REV = 746.6;
    static final double WHEEL_DIAMETER = 10.16;
    static final double Counts_PER_CM = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        //Gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Motors mapping
        leftDrive1  = hardwareMap.get(DcMotor.class, "left1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right2");

        claw = hardwareMap.get(Servo.class,"claw");

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        //Reset
        resetEncoders();
        resetGyro();

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //Robot code
        encoderDrive(0.5,50);
        //resetGyro();
        claw.setPosition(1);
        sleep(3000);
        claw.setPosition(0);
        sleep(3000);
        claw.setPosition(-1);
        sleep(3000);

        turn(true,90);

        /*while (opModeIsActive()){
            telemetry.addData("Angle", getAngle());
            telemetry.update();
        }*/

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

    private void resetEncoders(){
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Movements
    private void encoderDrive(double power, double distanceCM) {
        int target=((int)(distanceCM) * (int)(Counts_PER_CM));

        leftDrive1.setTargetPosition(target);
        leftDrive2.setTargetPosition(target);
        rightDrive1.setTargetPosition(target);
        rightDrive2.setTargetPosition(target);

        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && leftDrive1.isBusy() && leftDrive2.isBusy()
                && rightDrive1.isBusy() && rightDrive2.isBusy()) {
            motorSet(power, power);
        }
        motorStop();
    }

    private void turn(boolean left, double target) {
        resetGyro();
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (left) {
            while (opModeIsActive() && getAngle() <= target) {
                motorSet(-0.5, 0.5);
            }
            motorSet(0, 0);
        }
        else {
            while (opModeIsActive() && getAngle() >= target) {
                motorSet(0.5, -0.5);
            }
            motorSet(0, 0);
        }
    }

    //Gyro
    public void resetGyro(){
        last_angle  = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZXY, AngleUnit.DEGREES);
        angle = 0;
    }

    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle - last_angle.firstAngle;
        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;
        return angle;

    }
}

