package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PIDStrafeMotorEX", group="Linear Opmode")

public class PIDStrafeDrive extends LinearOpMode {

    // Declare OpMode members
    private DcMotorEx left1 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right2 = null;

    //PID
    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(40,0,1.7,0);
    PIDFCoefficients pidOrig,currentPID;

    //Encoder setup
    static final double     COUNTS_PER_MOTOR_REV = 704.86 ;
    static final double     DRIVE_GEAR_REDUCTION = 1 ;
    static final double     WHEEL_DIAMETER = 4.0 ;

    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415);
    static final double     Counts_PER_STRAFE = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER * 3.1415) * 0.7874015748;

    //Gyro setup
    public static double    drivecoeff = 0.003;
    public static double    strafecoeff = 0.004;

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
        PIDDrive(0.5,48,0);
        PIDStrafe(0.5,24,0);
    }

    //Main Functions
    private void PIDDrive(double power, double distance, double angle) {

        //Declare Variables
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        //Reduce Error
        moveCounts = (int)(distance * COUNTS_PER_INCH);
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
            error = getErrorAngle(angle);
            steer = getSteer(error, drivecoeff);

            //Angle Controller
            leftSpeed = power - steer;
            rightSpeed = power + steer;

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
    private void PIDStrafe(double power, double distance, double angle) {

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

        if (opModeIsActive()) {

            moveCounts = (int) (distance * Counts_PER_STRAFE);

            newLeft1Target = left1.getCurrentPosition() - moveCounts;
            newLeft2Target = left2.getCurrentPosition() + moveCounts;

            newRight1Target = right1.getCurrentPosition() + moveCounts;
            newRight2Target = right2.getCurrentPosition() - moveCounts;

            left1.setTargetPosition(newLeft1Target);
            left2.setTargetPosition(newLeft2Target);
            right1.setTargetPosition(newRight1Target);
            right2.setTargetPosition(newRight2Target);

            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            power = Range.clip(Math.abs(power), 0.0, 1.0);
            motorSet(power, power);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (left1.isBusy() && right1.isBusy()) && left2.isBusy() && right2.isBusy()) {

                error = getErrorAngle(angle);
                steer = getSteer(error, strafecoeff);

                if (distance < 0)
                    steer *= -1.0;

                left1Speed = power + steer;
                left2Speed = power - steer;

                right1Speed = power + steer;
                right2Speed = power - steer;

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

                left1.setPower(left1Speed);
                left2.setPower(left2Speed);

                right1.setPower(right1Speed);
                right2.setPower(right2Speed);
            }
        }
    }

    //Gyro Functions
    private double getErrorAngle(double targetAngle) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //Shortcuts
    private void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetGyro(){
        last_angle  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angle = 0;
    }
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
}
