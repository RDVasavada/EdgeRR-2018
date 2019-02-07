package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class EdgeBot {

    // Declare drive motors
    protected DcMotorEx leftDriveMotor;
    protected DcMotorEx rightDriveMotor;

    // Declare functional motors
    protected DcMotorEx liftMotor1;
    protected DcMotorEx liftMotor2;

    protected DcMotorEx boomExtendMotor;
    protected DcMotorEx boomRotateMotor;
    protected DcMotorEx boomAngleMotor;

    protected DcMotorEx deploymentMotor; // 600 counts
    int initialDeployCount = 0;

    // Declare other servos
    private CRServo leftSweeperServo;
    private CRServo rightSweeperServo;

    private Servo flipServo;

    private Servo liftServo;

    // Declare distance sensors
    private DistanceSensor markerDistanceSensor;
    private DistanceSensor liftDistanceSensor;

    // Declare imu (inertial motion unit)
    protected BNO055IMU imu;

    // Local OpMode members
    private HardwareMap hMap;
    private ElapsedTime localPeriod = new ElapsedTime();

    // External Camera
    //public Camera externalCamera = null;

    // Vision
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    // A reference to the current opMode
    public LinearOpMode currentOpmode;

    // Encoder values for the arm
    private int extendHome;
    private int rotateHome;
    private int angleHome;

    private int extendSilver;
    private int rotateSilver;
    private int angleSilver;

    private int extendGold;
    private int rotateGold;
    private int angleGold;

    protected boolean homeSet = false;
    protected boolean silverSet = false;
    protected boolean goldSet = false;

    // Constructor
    public EdgeBot() {

    }

    // Initializes the hardware interfaces -- ONLY call this in runOpMode()
    public void init(HardwareMap map, LinearOpMode opMode) {
        this.hMap = map;

        currentOpmode = opMode;

        // Initialize servos
        leftSweeperServo = hMap.crservo.get("lsweeper");
        rightSweeperServo = hMap.crservo.get("rsweeper");

        flipServo = hMap.servo.get("flipservo");

        liftServo = hMap.servo.get("liftservo");

        // Initialize drive motors
        leftDriveMotor = (DcMotorEx) hMap.dcMotor.get("ldmotor");
        rightDriveMotor = (DcMotorEx) hMap.dcMotor.get("rdmotor");

        // Initialize functional motors
        liftMotor1 = (DcMotorEx) hMap.dcMotor.get("liftmotor1");
        liftMotor1.setTargetPositionTolerance(15);

        liftMotor2 = (DcMotorEx) hMap.dcMotor.get("liftmotor2");
        liftMotor2.setTargetPositionTolerance(15);

        boomExtendMotor = (DcMotorEx) hMap.dcMotor.get("extendmotor");
        boomRotateMotor = (DcMotorEx) hMap.dcMotor.get("rotatemotor");
        boomAngleMotor = (DcMotorEx) hMap.dcMotor.get("anglemotor");

        deploymentMotor = (DcMotorEx) hMap.dcMotor.get("deploymotor");
        initialDeployCount = deploymentMotor.getCurrentPosition();

        // Initialize the distance sensors
        //marketDistanceSensor = hMap.get(DistanceSensor.class, "markerrange");
        //liftDistanceSensor = hMap.get(DistanceSensor.class, "liftrange");

        // Initialize the imu
        imu = hMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);
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

    public void mecanumDrive(double drive, double rotate) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsRunUsingEncoders();

        double leftPower = drive + rotate;
        double rightPower = drive - rotate;

        if (Math.abs(leftPower) > 1) {
            rightPower /= leftPower;
            leftPower /= leftPower;
        } else if (Math.abs(rightPower) > 1) {
            leftPower /= rightPower;
            rightPower /= rightPower;
        }

        leftDriveMotor.setPower(leftPower * 0.75);
        rightDriveMotor.setPower(rightPower * 0.75);
    }

    public void tankDrive(double drive, double rotate) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsRunUsingEncoders();

        double leftPower = Math.pow(drive, 2);
        double rightPower = Math.pow(drive, 2);

        if (drive < 0) {
            leftPower *= -1;
            rightPower *= -1;
        }

        leftPower += rotate / 2;
        rightPower -= rotate / 2;

        leftPower /= 1.5;
        rightPower /= 1.5;

        if (Math.abs(leftPower) > 1) {
            rightPower /= leftPower;
            leftPower /= leftPower;
        } else if (Math.abs(rightPower) > 1) {
            leftPower /= rightPower;
            rightPower /= rightPower;
        }

        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);
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

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    // Drive forwards at a given speed
    public void driveForwards(double speed) {
        setDriveMotorsRunUsingEncoders();

        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    public void driveBackwardForSteps(int numberOfSteps, double maxSpeed, Telemetry telemetry) {
        // Set motor directions
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Calculate step counts
        int leftDriveMotorTarget = leftDriveMotor.getCurrentPosition() + numberOfSteps;
        int rightDriveMotorTarget = rightDriveMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        leftDriveMotor.setTargetPosition(leftDriveMotorTarget);
        rightDriveMotor.setTargetPosition(rightDriveMotorTarget);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        leftDriveMotor.setPower(maxSpeed);
        rightDriveMotor.setPower(maxSpeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        stopDriveMotors();
    }

    public void driveForwardForSteps(int numberOfSteps, double maxSpeed, Telemetry telemetry) {
        // Set motor directions
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int leftDriveMotorTarget = leftDriveMotor.getCurrentPosition() + numberOfSteps;
        int rightDriveMotorTarget = rightDriveMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        leftDriveMotor.setTargetPosition(leftDriveMotorTarget);
        rightDriveMotor.setTargetPosition(rightDriveMotorTarget);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        leftDriveMotor.setPower(maxSpeed);
        rightDriveMotor.setPower(maxSpeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        stopDriveMotors();
    }

    public void driveBackwardForInches(double inches, double speed, Telemetry telemetry) {
        int steps = inchToEncoder(inches);

        driveBackwardForSteps(steps, speed, telemetry);
    }

    public void driveForwardForInches(double inches, double speed, Telemetry telemetry) {
        int steps = inchToEncoder(inches);

        driveForwardForSteps(steps, speed, telemetry);
    }

    // Rotate counterclockwise at given speed
    public void rotateCounterClockwise(double speed) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    // Rotate clockwise at given speed
    public void rotateClockwise(double speed) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    public void rotateCounterClockwiseGyro(int degreesToRotate, double maxSpeed, Telemetry telemetry) {
        // Record initial heading
        double initialHeading = getRawGyroHeading();

        // Calculate target heading
        double targetHeading = initialHeading + degreesToRotate;

        if (targetHeading > 180) {
            targetHeading -= 360;
        } else if (targetHeading < -180) {
            targetHeading += 360;
        }

        setDriveMotorsRunUsingEncoders();

        double currentHeading = initialHeading;

        double error = targetHeading - currentHeading;

        if (error < -180) {
            error += 360;
        }

        localPeriod.reset();

        while (Math.abs(error) > 3 && currentOpmode.opModeIsActive() && localPeriod.milliseconds() < 7000) {
            currentHeading = getRawGyroHeading();

            error = targetHeading - currentHeading;

            if (error < -180) {
                error += 360;
            }

            double speed = maxSpeed * (Math.abs(error) / 75) + 0.3;
            if (speed > maxSpeed) {
                speed = maxSpeed;
            }

            rotateCounterClockwise(speed);
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
    }

    public void rotateClockwiseGyro(int degreesToRotate, double maxSpeed, Telemetry telemetry) {
        // Record initial heading
        double initialHeading = getRawGyroHeading();

        // Calculate target heading
        double targetHeading = initialHeading - degreesToRotate;

        if (targetHeading > 180) {
            targetHeading -= 360;
        } else if (targetHeading < -180) {
            targetHeading += 360;
        }

        setDriveMotorsRunUsingEncoders();

        double currentHeading = initialHeading;

        double error = targetHeading - currentHeading;

        if (error > 180) {
            error -= 360;
        }

        localPeriod.reset();

        while (Math.abs(error) > 3 && currentOpmode.opModeIsActive() && localPeriod.milliseconds() < 7000) {
            currentHeading = getRawGyroHeading();

            error = targetHeading - currentHeading;

            if (error > 180) {
                error -= 360;
            }

            double speed = maxSpeed * (Math.abs(error) / 75) + 0.3;
            if (speed > maxSpeed) {
                speed = maxSpeed;
            }

            rotateClockwise(speed);
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
    }

    // Drive to a certain distance (inches) on the marker sensor
    public void driveToMarkerDistance(double power, double distance) {
        double currentDistance = getMarkerSensorDistance(DistanceUnit.INCH);

        double error = distance - currentDistance;

        while (currentOpmode.opModeIsActive() && Math.abs(error) > 1.5) {
            if (error > 0) {
                driveForwards(power);
            } else {
                driveBackwards(power);
            }
        }
    }

    // Drive to a certain distance (inches) on the lift sensor
    public void driveToLiftDistance(double power, double distance) {
        double currentDistance = getLiftSensorDistance(DistanceUnit.INCH);

        double error = distance - currentDistance;

        while (currentOpmode.opModeIsActive() && Math.abs(error) > 1.5) {
            if (error > 0) {
                driveBackwards(power);
            } else {
                driveForwards(power);
            }
        }
    }

    // Set the drive motors to run without encoders
    public void setDriveMotorsRunWithoutEncoders() {
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

    // Set the arm motors to reset encoders
    public void setArmMotorsResetEncoders() {
        boomExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boomAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boomRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Set the arm motors to run to position
    public void setArmMotorsRunToPosition() {
        boomExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void robotLowerAuton() {
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int liftMotor1Target = liftMotor2.getCurrentPosition() + 16500;
        int liftMotor2Target = liftMotor2.getCurrentPosition() + 16500;

        // Set target steps
        liftMotor1.setTargetPosition(liftMotor1Target);
        liftMotor2.setTargetPosition(liftMotor2Target);

        // Turn on RUN_TO_POSITION
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (liftMotor1.isBusy() && liftMotor2.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        }

        // Stop all motion;
        liftMotor2.setPower(0);

        // Turn off RUN_TO_POSITION
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void robotLift(double power) {
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor1.setPower(power);

        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor2.setPower(power);
    }

    public void armHomePosition() {
        if (goldSet && homeSet) {
            setArmMotorsRunToPosition();

            int extendDistance = extendGold - extendHome;
            int angleDistance = angleGold - angleHome;
            int rotateDistance = rotateGold - rotateHome;

            boomRotateMotor.setTargetPosition(rotateHome);

            if (boomRotateMotor.getCurrentPosition() > (rotateHome + rotateDistance * 0.7)) {
                boomRotateMotor.setPower(0.6);

                boomExtendMotor.setTargetPosition(extendSilver);
                boomExtendMotor.setPower(0.3);
            } else if (boomRotateMotor.getCurrentPosition() > (rotateHome + rotateDistance / 3)) {
                boomRotateMotor.setPower(0.7);

                boomAngleMotor.setTargetPosition(angleHome);
                boomAngleMotor.setPower(1);
            } else {
                boomRotateMotor.setPower(0.2);

                boomExtendMotor.setTargetPosition(extendHome);
                boomExtendMotor.setPower(0.15);
            }

            if (boomRotateMotor.getCurrentPosition() > (rotateHome + rotateDistance * 0.7)) {
                boomRotateMotor.setPower(0.8);
            } else if (boomRotateMotor.getCurrentPosition() > (rotateHome + rotateDistance * 0.4)) {
                boomRotateMotor.setPower(0.6);
            } else {
                boomRotateMotor.setPower(0.2);
            }
        }
    }

    public void armSilverPosition() {
        if (goldSet && homeSet) {
            setArmMotorsRunToPosition();

            int extendDistance = extendSilver - extendHome;
            int angleDistance = angleSilver - angleHome;
            int rotateDistance = rotateSilver - rotateHome;

            boomRotateMotor.setTargetPosition(rotateSilver);

            if (boomRotateMotor.getCurrentPosition() < (rotateHome + rotateDistance * 0.8)) {
                boomAngleMotor.setTargetPosition(angleSilver);
                boomAngleMotor.setPower(1);

                boomExtendMotor.setTargetPosition(extendSilver);
                boomExtendMotor.setPower(0.3);
            } else {
                boomAngleMotor.setTargetPosition(angleSilver);
                boomAngleMotor.setPower(1);

                boomExtendMotor.setTargetPosition(extendSilver);
                boomExtendMotor.setPower(0.15);
            }

            if (boomRotateMotor.getCurrentPosition() < (rotateHome + rotateDistance * 0.7)) {
                boomRotateMotor.setPower(0.8);
            } else {
                boomRotateMotor.setPower(0.2);
            }
        }
    }

    public void armGoldPosition(Telemetry telemetry) {
        if (goldSet && homeSet) {
            setArmMotorsRunToPosition();

            int extendDistance = extendGold - extendHome;
            int angleDistance = angleGold - angleHome;
            int rotateDistance = rotateGold - rotateHome;

            boomRotateMotor.setTargetPosition(rotateGold);

            if (boomRotateMotor.getCurrentPosition() < (rotateHome + rotateDistance * 0.8)) {
                boomAngleMotor.setTargetPosition(angleGold);
                boomAngleMotor.setPower(1);

                boomExtendMotor.setTargetPosition(extendHome - 450);
                boomExtendMotor.setPower(0.3);
            } else {
                boomAngleMotor.setTargetPosition(angleGold);
                boomAngleMotor.setPower(1);

                boomExtendMotor.setTargetPosition(extendGold);
                boomExtendMotor.setPower(0.15);
            }

            if (boomRotateMotor.getCurrentPosition() < (rotateHome + rotateDistance * 0.7)) {
                boomRotateMotor.setPower(0.8);
            } else {
                boomRotateMotor.setPower(0.2);
            }
        }
    }

    public void setArmHome() {
        extendHome = boomExtendMotor.getCurrentPosition();
        rotateHome = boomRotateMotor.getCurrentPosition();
        angleHome = boomAngleMotor.getCurrentPosition();

        homeSet = true;
    }

    public void setArmSilver() {
        extendSilver = boomExtendMotor.getCurrentPosition();
        rotateSilver = boomRotateMotor.getCurrentPosition();
        angleSilver = boomAngleMotor.getCurrentPosition();

        silverSet = true;
    }

    public void setArmGold() {
        extendGold = boomExtendMotor.getCurrentPosition();
        rotateGold = boomRotateMotor.getCurrentPosition();
        angleGold = boomAngleMotor.getCurrentPosition();

        goldSet = true;
    }

    public void boomExtend(double power) {
        boomExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomExtendMotor.setPower(power);
    }

    public void boomExtendStop() {
        boomExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomExtendMotor.setTargetPosition(boomExtendMotor.getCurrentPosition());
        boomExtendMotor.setPower(0.5);
    }

    public void boomRotate(double power) {
        boomRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomRotateMotor.setPower(power);
    }

    public void boomRotateAuton() {
        boomRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomRotateMotor.setTargetPosition(boomRotateMotor.getCurrentPosition() + 180);
        boomRotateMotor.setPower(0.5);

        while (currentOpmode.opModeIsActive() && boomRotateMotor.isBusy()) {
            waitForTick(50);
        }

        boomRotateMotor.setPower(0);

        boomRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void dropMarker() {
        boomRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void boomAngle(double power) {
        boomAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomAngleMotor.setPower(power);
    }

    public void runDeployMotor(double power) {
        if (power > 0) {
            if (deploymentMotor.getCurrentPosition() <= initialDeployCount) {
                deploymentMotor.setPower(power);
            }
        }

        if (power < 0) {
            if (deploymentMotor.getCurrentPosition() > initialDeployCount - 600) {
                deploymentMotor.setPower(power);
            }
        } else {
            deploymentMotor.setPower(0);
        }
    }

    public void leftIntakeSweep() {
        leftSweeperServo.setPower(-1);
    }

    public void leftIntakeStop() {
        leftSweeperServo.setPower(0);
    }

    public void leftIntakeReverse() {
        leftSweeperServo.setPower(1);
    }

    public void rightIntakeSweep() {
        rightSweeperServo.setPower(1);
    }

    public void rightIntakeStop() {
        rightSweeperServo.setPower(0);
    }

    public void rightIntakeReverse() {
        rightSweeperServo.setPower(-1);
    }

    public void flipServoDown() {
        flipServo.setPosition(0);
    }

    public void flipServoUp() {
        flipServo.setPosition(0.5);
    }

    public void liftServoRelease() {
        liftServo.setPosition(0.6);
    }

    public void liftServoClamp() {
        liftServo.setPosition(0);
    }

    // Get the distance sensors values
    public double getMarkerSensorDistance(DistanceUnit unit) {
        return markerDistanceSensor.getDistance(unit);
    }

    public double getLiftSensorDistance(DistanceUnit unit) {
        return liftDistanceSensor.getDistance(unit);
    }

    // Get the raw gyro heading in degrees
    public float getRawGyroHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void calibrateGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            waitForTick(20);
        }
    }

    // Convert from inches to encoder counts
    public int inchToEncoder(double inches) {
        int counts = (int) Math.round(inches * Constants.Chassis.COUNTS_PER_INCH_M); // 37.4 encoder counts for one inch

        return counts;
    }

    public cubeLocation detectCube(Telemetry telemetry) {
        cubeLocation location = cubeLocation.UNKNOWN;

        boolean goldSeen = false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() > 1) {

                    double silverX = -1;
                    double goldX = -1;

                    int silverCount = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == Constants.Vision.LABEL_GOLD_MINERAL) {
                            telemetry.addData("Confidence", recognition.getConfidence());
                            if (recognition.getConfidence() > 0.75) {
                                goldSeen = true;
                                goldX = recognition.getRight();
                                telemetry.addData("Gold right", goldX);
                            }
                        } else if (recognition.getLabel() == Constants.Vision.LABEL_SILVER_MINERAL) {
                            silverX = recognition.getRight();
                            silverCount++;
                        }
                    }

                    if (!goldSeen) {
                        if (silverCount == 2) {
                            location = cubeLocation.RIGHT;
                        }
                    } else {
                        if (silverCount == 1) {
                            if (goldX > silverX) {
                                location = cubeLocation.LEFT;
                            } else {
                                location = cubeLocation.CENTER;
                            }
                        }
                    }
                }
            }
        }

        telemetry.addData("Gold location: ", location);
        telemetry.update();

        return location;
    }

    public void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.Vision.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.Vision.TFOD_MODEL_ASSET, Constants.Vision.LABEL_GOLD_MINERAL, Constants.Vision.LABEL_SILVER_MINERAL);
    }

    public void activateTfod() {
        tfod.activate();
    }

    public void shutdownTfod() {
        tfod.shutdown();
    }

}

enum cubeLocation {
    LEFT, CENTER, RIGHT, UNKNOWN;
}

