package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class EdgeDriveServo {
    // The servo
    private CRServo servo;

    // The potentiometer
    private AnalogInput potentiometer;

    // Current power of the servo
    private double currentPower;

    public EdgeDriveServo(CRServo crServo, AnalogInput pot) {
        // Initialize the servo and potentiometer
        servo = crServo;
        potentiometer = pot;

        // Set the current power to zero
        currentPower = 0;
    }

    public void setPower(double power) {
        servo.setPower(power);
        currentPower = power;
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    public double getPositionDegrees() {
        return potentiometer.getVoltage() * 360 / 3.3;
    }

    public boolean isBusy() {
        return (currentPower != 0) ? true : false;
    }
}
