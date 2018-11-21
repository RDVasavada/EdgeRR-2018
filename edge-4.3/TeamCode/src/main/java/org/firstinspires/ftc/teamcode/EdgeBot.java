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
        frontLeftServo = new EdgeDriveServo(hMap.crservo.get("flservo"), hMap.analogInput.get("flpot"));
        frontRightServo = new EdgeDriveServo(hMap.crservo.get("frservo"), hMap.analogInput.get("frpot"));
        rearLeftServo = new EdgeDriveServo(hMap.crservo.get("rlservo"), hMap.analogInput.get("rlpot"));
        rearRightServo = new EdgeDriveServo(hMap.crservo.get("rrservo"), hMap.analogInput.get("rrpot"));

        // Initialize drive motors
        leftDriveMotor = hMap.dcMotor.get("ldmotor");
        rightDriveMotor = hMap.dcMotor.get("rdmotor");

        // Initialize functional motors
        liftMotor = hMap.dcMotor.get("liftmotor");

        boomExtendMotor = hMap.dcMotor.get("extendmotor");
        boomRotateMotor = hMap.dcMotor.get("rotatemotor");
        boomAngleMotor = hMap.dcMotor.get("anglemotor");
        particlePickupMotor = hMap.dcMotor.get("intakemotor");
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

    public void swerveDrive(double drive, double rotate, double leftPower, double rightPower) {
        if (Math.abs(rotate) > 0.05) {
            for (int i = 0; i < 4; i++) {
                driveServos[i].setPower(rotate);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                double degrees = driveServos[i].getPositionDegrees();
                double difference = frontLeftServo.getPositionDegrees() - degrees;
                double scaled = difference / 40;

                if (Math.abs(difference) > 1) {
                    if (Math.abs(scaled) > 1) {
                        scaled /= 1;
                    } else if (Math.abs(scaled) < 0.02) {
                        scaled = 0;
                    }
                } else {
                    scaled = 0;
                }

                driveServos[i].setPower(scaled);
            }
        }

        tankDrive(drive, rotate);
    }

    public void tankDrive(double drive, double rotate) {
        double leftPower = drive + rotate;
        double rightPower = drive - rotate;

        if (leftPower > 1) {
            rightPower /= leftPower;
            leftPower = 1;
        }

        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);
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

}