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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    public DcMotor leftDriveMotor;
    public DcMotor rightDriveMotor;

    // Declare functional motors
    public DcMotor liftMotor;

    public DcMotor boomExtendMotor;
    private DcMotor boomRotateMotor;
    public DcMotor boomAngleMotor;

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

    private Servo liftServo;

    // Declare distance sensors
    private DistanceSensor bottomDistanceSensor;
    private DistanceSensor topDistanceSensor;

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
        leftClampServo.scaleRange(0.1, 0.9);

        rightClampServo = hMap.servo.get("rcservo");
        rightClampServo.scaleRange(0.1, 0.9);

        liftServo =  hMap.servo.get("liftservo");

        // Initialize drive motors
        leftDriveMotor = hMap.dcMotor.get("ldmotor");
        rightDriveMotor = hMap.dcMotor.get("rdmotor");

        // Initialize functional motors
        liftMotor = hMap.dcMotor.get("liftmotor");

        boomExtendMotor = hMap.dcMotor.get("extendmotor");

        boomRotateMotor = hMap.dcMotor.get("rotatemotor");

        boomAngleMotor = hMap.dcMotor.get("anglemotor");

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

        // Start motion
        double lowSpeed = 0.1;

        leftDriveMotor.setPower(maxSpeed);
        rightDriveMotor.setPower(maxSpeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);

            double stepsToDo = ((leftDriveMotorTarget - leftDriveMotor.getCurrentPosition()) + (rightDriveMotorTarget - rightDriveMotor.getCurrentPosition())) / 2;
            double progress = (numberOfSteps - stepsToDo) / numberOfSteps;

            double currentSpeed = Math.sin((Math.PI) * progress) * maxSpeed + lowSpeed;
            if (currentSpeed > maxSpeed) {
                currentSpeed = maxSpeed;
            }

            telemetry.addData("progress", progress);
            telemetry.addData("speed", currentSpeed);
            telemetry.update();

            leftDriveMotor.setPower(maxSpeed);
            rightDriveMotor.setPower(maxSpeed);
        }
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

        // Start motion
        double lowSpeed = 0.1;

        leftDriveMotor.setPower(maxSpeed);
        rightDriveMotor.setPower(maxSpeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (leftDriveMotor.isBusy() && rightDriveMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);

            double stepsToDo = ((leftDriveMotorTarget - leftDriveMotor.getCurrentPosition()) + (rightDriveMotorTarget - rightDriveMotor.getCurrentPosition())) / 2;
            double progress = (numberOfSteps - stepsToDo) / numberOfSteps;

            double currentSpeed = Math.sin((Math.PI) * progress) * maxSpeed + lowSpeed;
            if (currentSpeed > maxSpeed) {
                currentSpeed = maxSpeed;
            }

            telemetry.addData("progress", progress);
            telemetry.addData("speed", maxSpeed);
            telemetry.addData("left encoder steps to do", leftDriveMotorTarget - leftDriveMotor.getCurrentPosition());
            telemetry.addData("right encoder steps to do", rightDriveMotorTarget - leftDriveMotor.getCurrentPosition());

            telemetry.update();

            leftDriveMotor.setPower(maxSpeed);
            rightDriveMotor.setPower(maxSpeed);
        }
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

    public void rotateCounterClockwiseGyro(int degreesToRotate, double maxSpeed) {
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
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
    }

    public void rotateClockwiseGyro(int degreesToRotate, double maxSpeed) {
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
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void robotLift(double power) {
        liftMotor.setPower(power);
    }

    public void boomExtend(double power) {
        boomExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomExtendMotor.setPower(power);
    }

    public void boomExtendStop() {
        boomExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomExtendMotor.setTargetPosition(boomExtendMotor.getCurrentPosition());
        boomExtendMotor.setPower(0.6);
    }

    public void boomRotate(double power) {
        boomRotateMotor.setPower(power);
    }

    public void boomAngle(double power) {
        boomAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomAngleMotor.setPower(power);
    }

    public void boomAngleStop() {
        boomAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boomAngleMotor.setTargetPosition(boomAngleMotor.getCurrentPosition());
        boomAngleMotor.setPower(1);
    }

    public void leftServoRelease() {
        leftClampServo.setPosition(1);
    }

    public void leftServoClamp() {
        leftClampServo.setPosition(0);
    }

    public void rightServoRelease() {
        rightClampServo.setPosition(1);
    }

    public void rightServoClamp() {
        rightClampServo.setPosition(0);
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

    // Convert from inches to encoder counts
    public int inchToEncoder(double inches) {
        int counts = (int)Math.round(inches * Constants.Chassis.COUNTS_PER_INCH); // 39 encoder counts for one inch

        return counts;
    }

}