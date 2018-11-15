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
    // Declare motors
    public DcMotor forwardDriveMotor;
    public DcMotor reverseDriveMotor;

    // Declare drive servos
    public EdgeDriveServo servo;
    public EdgeDriveServo frontLeftServo;
    public EdgeDriveServo frontRightServo;
    public EdgeDriveServo rearLeftServo;
    public EdgeDriveServo rearRightServo;

    // Array of drive servos
    public EdgeDriveServo[] driveServos = {frontLeftServo, frontRightServo, rearLeftServo, rearRightServo};

    // Declare motors
    public DcMotor tankTurretLiftMotor;
    public DcMotor tankTurretExtendMotor;
    public DcMotor tankTurretRotateMotor;
    public DcMotor particlePickupMotor;

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

        servo = new EdgeDriveServo(hMap.crservo.get("servo"), hMap.analogInput.get("pot"));
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

    public void swerveDrive(double driveSpeed, double rotate) {
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
    }

    public void tankTurretLift(double power) {

    }

    public void tankTurretExtend(double power) {

    }

    public void tankTurretRotate(double power) {

    }

    public void particlePickup(double power) {

    }

}