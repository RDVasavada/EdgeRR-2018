package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class EdgeBot {
    // Declare drive motors
    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;

    // Declare functional motors
    private DcMotor liftMotor;

    private DcMotor boomExtendMotor;
    private DcMotor boomRotateMotor;
    private DcMotor boomAngleMotor;
    private DcMotor particlePickupMotor;

    // Declare drive servos
    private EdgeDriveServo frontLeftServo;
    private EdgeDriveServo frontRightServo;
    private EdgeDriveServo rearLeftServo;
    private EdgeDriveServo rearRightServo;

    // Array of drive servos
    private EdgeDriveServo[] driveServos = {frontLeftServo, frontRightServo, rearLeftServo, rearRightServo};

    // Declare other servos
    private Servo leftClampServo;
    private Servo rightClampServo;

    // Declare imu (inertial motion unit)
    public BNO055IMU imu;

    // Local OpMode members
    private HardwareMap hMap;
    private ElapsedTime localPeriod = new ElapsedTime();

    // External Camera
    //public Camera externalCamera = null;

    // A reference to the current opMode
    public LinearOpMode currentOpmode;

    // Constructor
    public EdgeBot() {

    }

    // Initializes the hardware interfaces -- ONLY call this in runOpMode()
    public void init(HardwareMap map, LinearOpMode opMode) {
        this.hMap = map;

        currentOpmode = opMode;

        // Initialize servos
        //frontLeftServo = new EdgeDriveServo(hMap.crservo.get("flservo"), hMap.analogInput.get("flpot"));
        //frontRightServo = new EdgeDriveServo(hMap.crservo.get("frservo"), hMap.analogInput.get("frpot"));
        //rearLeftServo = new EdgeDriveServo(hMap.crservo.get("rlservo"), hMap.analogInput.get("rlpot"));
        //rearRightServo = new EdgeDriveServo(hMap.crservo.get("rrservo"), hMap.analogInput.get("rrpot"));

        leftClampServo = hMap.servo.get("lcservo");
        rightClampServo = hMap.servo.get("rcservo");

        // Initialize drive motors
        leftDriveMotor = hMap.dcMotor.get("ldmotor");
        rightDriveMotor = hMap.dcMotor.get("rdmotor");

        // Initialize functional motors
        liftMotor = hMap.dcMotor.get("liftmotor");

        boomExtendMotor = hMap.dcMotor.get("extendmotor");
        boomRotateMotor = hMap.dcMotor.get("rotatemotor");
        boomRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boomAngleMotor = hMap.dcMotor.get("anglemotor");
        /*
        particlePickupMotor = hMap.dcMotor.get("intakemotor");

        // Initialize the imu
        imu = hMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);
        */
    }

    // Waits until a certain time has elapsed since the last call
    public void waitForTick(long periodMilliseconds) {
        long remaining = periodMilliseconds - (long) localPeriod.milliseconds();

        // Sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        localPeriod.reset();
    }

    public void swerveDrive(double drive, double rotate, double swerve) {
        if (Math.abs(rotate) > 0.05) {
            for (int i = 0; i < 4; i++) {
                driveServos[i].setPower(swerve);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                driveServos[i].setPower(0);
            }
        }

        tankDrive(drive, rotate);
    }

    public void tankDrive(double drive, double rotate) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double leftPower = drive + rotate;
        double rightPower = drive - rotate;

        if (leftPower > 1) {
            rightPower /= leftPower;
            leftPower = 1;
        } else if (rightPower > 1) {
            leftPower /= rightPower;
            rightPower = 1;
        }

        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);
    }

    public void traditionalTankDrive(double left, double right) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveMotor.setPower(left);
        rightDriveMotor.setPower(right);
    }

    // Stop all of the drive motors
    public void stopDriveMotors() {
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    // Drive backwards at a given speed
    public void driveBackwards(double speed) {
        setDriveMotorsRunUsingEncoders();

        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsToCommonSpeed(speed);
    }

    // Drive forwards at a given speed
    public void driveForwards(double speed) {
        setDriveMotorsRunUsingEncoders();

        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMotorsToCommonSpeed(speed);
    }

    public void driveBackwardForSteps(int numberOfSteps, double speed) {
        // Set motor directions
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int leftDriveMotorStepsToDo = leftDriveMotor.getCurrentPosition() + numberOfSteps;
        int rightDriveMotorStepsToDo = rightDriveMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        leftDriveMotor.setTargetPosition(leftDriveMotorStepsToDo);
        rightDriveMotor.setTargetPosition(rightDriveMotorStepsToDo);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // Start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion;
        stopDriveMotors();

        // Turn off RUN_TO_POSITION
        setDriveMotorsRunUsingEncoders();
    }

    public void driveForwardForSteps(int numberOfSteps, double speed) {
        // Set motor directions
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int leftDriveMotorStepsToDo = leftDriveMotor.getCurrentPosition() + numberOfSteps;
        int rightDriveMotorStepsToDo = rightDriveMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        leftDriveMotor.setTargetPosition(leftDriveMotorStepsToDo);
        rightDriveMotor.setTargetPosition(rightDriveMotorStepsToDo);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // Start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion;
        stopDriveMotors();

        // Turn off RUN_TO_POSITION
        setDriveMotorsRunUsingEncoders();
    }

    public void driveBackwardForInches(double inches, double speed) {
        int steps = inchToEncoder(inches);

        driveBackwardForSteps(steps, speed);
    }

    public void driveForwardForInches(double inches, double speed) {
        int steps = inchToEncoder(inches);

        driveForwardForSteps(steps, speed);
    }

    // Set motors to common speed
    public void setDriveMotorsToCommonSpeed(double speed) {
        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    // Set the drive motors to run without encoders
    public  void setDriveMotorsRunWithoutEncoders() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the drive motors to run using encoders
    public void setDriveMotorsRunUsingEncoders() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set the drive motors mode to stop and reset encoders
    public void setDriveMotorsResetEncoders() {
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Set the drive motors to run to a given position
    public void setDriveMotorsRunToPosition() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void robotLift(double power) {
        liftMotor.setPower(power);
    }

    public void boomExtend(double power) {
        boomExtendMotor.setPower(power);
    }

    public void boomRotate(double power) {
        boomRotateMotor.setPower(power);
    }

    public void boomAngle(double power) {
        boomAngleMotor.setPower(power);
    }

    public void particleOut(double power) {
        particlePickupMotor.setPower(power);
    }

    public void particleIn(double power) {
        particlePickupMotor.setPower(-power);
    }

    public void servoClamp() {
        leftClampServo.setPosition(0);
        rightClampServo.setPosition(0);
    }

    public void servoMid() {
        leftClampServo.setPosition(0.5);
        rightClampServo.setPosition(0.5);
    }

    public void servoRelease() {
        leftClampServo.setPosition(1);
        rightClampServo.setPosition(1);
    }

    public void rotateWheelsToHeading(double targetHeading) {
        while (currentOpmode.opModeIsActive() && frontLeftServo.isBusy() && frontRightServo.isBusy() && rearLeftServo.isBusy() && rearRightServo.isBusy()) {
            for (int i = 0; i < 4; i++) {
                double currentHeading = driveServos[i].getPositionDegrees();
                double error = targetHeading - currentHeading;
                double output;

                if (Math.abs(error) > 0.5) {
                    output = (error / 40) + (error > 0 ? 0.1 : -0.1);
                } else {
                    output = 0;
                }

                driveServos[i].setPower(output);
            }
        }
    }

    // Get the raw gyro heading in degrees
    public float getRawGyroHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Convert from inches to encoder counts
    public int inchToEncoder(double inches) {
        int counts = (int)Math.round(inches * Constants.Chassis.COUNTS_PER_INCH); // 39 encoder counts for one inch

        return counts;
    }

}