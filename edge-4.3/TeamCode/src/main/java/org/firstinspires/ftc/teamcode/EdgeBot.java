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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class EdgeBot {
    // Declare drive motors
    protected DcMotorEx leftDriveMotor;
    protected DcMotorEx rightDriveMotor;

    // Declare functional motors
    protected DcMotorEx liftMotor;

    protected DcMotorEx boomExtendMotor;
    protected DcMotorEx boomRotateMotor;
    protected DcMotorEx boomAngleMotor;

    protected DcMotorEx deploymentMotor;

    // Declare drive servos
    private EdgeDriveServo frontLeftServo;
    private EdgeDriveServo frontRightServo;
    private EdgeDriveServo rearLeftServo;
    private EdgeDriveServo rearRightServo;

    // Array of drive servos
    private EdgeDriveServo[] driveServos = {frontLeftServo, frontRightServo, rearLeftServo, rearRightServo};

    // Declare other servos
    private CRServo leftSweeperServo;
    private CRServo rightSweeperServo;

    private Servo flipServo;

    private Servo liftServo;

    // Declare distance sensors
    private DistanceSensor bottomDistanceSensor;
    private DistanceSensor topDistanceSensor;

    // Declare imu (inertial motion unit)
    protected BNO055IMU imu;

    // Local OpMode members
    private HardwareMap hMap;
    private ElapsedTime localPeriod = new ElapsedTime();
    private ElapsedTime armTimer = new ElapsedTime();

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
        //frontLeftServo = new EdgeDriveServo(hMap.crservo.get("flservo"), hMap.analogInput.get("flpot"));
        //frontRightServo = new EdgeDriveServo(hMap.crservo.get("frservo"), hMap.analogInput.get("frpot"));
        //rearLeftServo = new EdgeDriveServo(hMap.crservo.get("rlservo"), hMap.analogInput.get("rlpot"));
        //rearRightServo = new EdgeDriveServo(hMap.crservo.get("rrservo"), hMap.analogInput.get("rrpot"));

        leftSweeperServo = hMap.crservo.get("lsweeper");
        rightSweeperServo = hMap.crservo.get("rsweeper");

        flipServo = hMap.servo.get("flipservo");

        liftServo =  hMap.servo.get("liftservo");

        // Initialize drive motors
        leftDriveMotor = (DcMotorEx)hMap.dcMotor.get("ldmotor");
        rightDriveMotor = (DcMotorEx)hMap.dcMotor.get("rdmotor");

        // Initialize functional motors
        liftMotor = (DcMotorEx)hMap.dcMotor.get("liftmotor");
        boomExtendMotor = (DcMotorEx)hMap.dcMotor.get("extendmotor");
        boomRotateMotor = (DcMotorEx)hMap.dcMotor.get("rotatemotor");
        boomAngleMotor = (DcMotorEx)hMap.dcMotor.get("anglemotor");
        deploymentMotor = (DcMotorEx)hMap.dcMotor.get("deploymotor");

        // Initialize the distance sensors
        bottomDistanceSensor = hMap.get(DistanceSensor.class, "bottomrange");
        topDistanceSensor = hMap.get(DistanceSensor.class, "toprange");

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

    // Resets the arm timer
    public void resetArmTimer() {
        armTimer.reset();
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

        setDriveMotorsRunUsingEncoders();

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
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveMotor.setPower(speed);
        rightDriveMotor.setPower(speed);
    }

    // Rotate clockwise at given speed
    public void rotateClockwise(double speed) {
        leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        while (Math.abs(error) > 1 && currentOpmode.opModeIsActive()) {
            currentHeading = getRawGyroHeading();

            error = targetHeading - currentHeading;

            if (error < -180) {
                error += 360;
            }

            double speed = maxSpeed * (Math.abs(error) / 200) + 0.05;
            if (speed > maxSpeed) {
                speed = maxSpeed;
            }

            rotateCounterClockwise(speed);

            telemetry.addData("Initial heading", initialHeading);
            telemetry.addData("Current Heading", getRawGyroHeading());
            telemetry.addData("Target heading", targetHeading);
            telemetry.update();
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

        while (Math.abs(error) > 1 && currentOpmode.opModeIsActive()) {
            currentHeading = getRawGyroHeading();

            error = targetHeading - currentHeading;

            if (error > 180) {
                error -= 360;
            }

            double speed = maxSpeed * (Math.abs(error) / 200) + 0.05;
            if (speed > maxSpeed) {
                speed = maxSpeed;
            }

            rotateClockwise(speed);

            telemetry.addData("Initial heading", initialHeading);
            telemetry.addData("Current Heading", getRawGyroHeading());
            telemetry.addData("Target heading", targetHeading);
            telemetry.update();
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
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
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int liftMotorTarget = liftMotor.getCurrentPosition() + 15000;

        // Set target steps
        liftMotor.setTargetPosition(liftMotorTarget);

        // Turn on RUN_TO_POSITION
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (liftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
            liftMotor.setPower(0.1);
        }

        // Stop all motion;
        liftMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void robotLift(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(power);
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

    public void boomAngle(double power) {
        boomAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomAngleMotor.setPower(power);
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

    public void flipServoHold() {
        flipServo.setPosition(0);
    }

    public void flipServoFlip() {
        flipServo.setPosition(0.5);
    }

    public void liftServoRelease() {
        liftServo.setPosition(0);
    }

    public void liftServoClamp() {
        liftServo.setPosition(0.75);
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

    // Get the distance sensors values
    public double getBottomSensorDistance(DistanceUnit unit) {
        return bottomDistanceSensor.getDistance(unit);
    }

    public double getTopSensorDistance(DistanceUnit unit) {
        return topDistanceSensor.getDistance(unit);
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
        int counts = (int)Math.round(inches * Constants.Chassis.COUNTS_PER_INCH); // 37.4 encoder counts for one inch

        return counts;
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

}