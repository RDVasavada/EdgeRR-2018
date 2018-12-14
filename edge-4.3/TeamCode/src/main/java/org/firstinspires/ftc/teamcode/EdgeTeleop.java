/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Driver Control")
//@Disabled
public class EdgeTeleop extends LinearOpMode {

    EdgeBot robot;

    // For braking the arm motors
    boolean extensionMotorIsStopped;
    boolean angleMotorIsStopped;

    @Override
    public void runOpMode() {
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        int leftStartingCount = robot.leftDriveMotor.getCurrentPosition();
        int rightStartingCount = robot.rightDriveMotor.getCurrentPosition();


        while(opModeIsActive()) {
            /* *** GamePad One *** */
            double driveSpeed = gamepad1.left_stick_y;
            double rotateSpeed = gamepad1.left_stick_x;

            double liftSpeed = gamepad1.right_trigger - gamepad1.left_trigger;

            robot.tankDrive(driveSpeed, rotateSpeed);

            robot.robotLift(liftSpeed);
            telemetry.addData("Left count", robot.leftDriveMotor.getCurrentPosition() - leftStartingCount);
            telemetry.addData("Right count", robot.rightDriveMotor.getCurrentPosition() - rightStartingCount);

            if (gamepad1.a) {
                robot.liftServoClamp();
            } else if (gamepad1.b) {
                robot.liftServoRelease();
            }

            /* *** GamePad Two *** */
            double boomAngleSpeed = gamepad2.right_stick_y;
            double boomRotateSpeed = gamepad2.right_stick_x;
            double boomExtendSpeed = (gamepad2.right_trigger - gamepad2.left_trigger) * 0.5;

            if (!(Math.abs(boomAngleSpeed) < 0.1)) {
                robot.boomAngle(boomAngleSpeed);
                angleMotorIsStopped = false;
            } else if (!angleMotorIsStopped) {
                //robot.boomAngleStop();
                robot.boomAngle(0);
                angleMotorIsStopped = true;
            }

            telemetry.addData("Angle motor voltage", robot.boomAngleMotor.getPower());
            telemetry.addData("Angle motor current position", robot.boomAngleMotor.getCurrentPosition());
            telemetry.addData("Angle motor target", robot.boomAngleMotor.getTargetPosition());

            robot.boomRotate(boomRotateSpeed);

            if (boomExtendSpeed != 0) {
                robot.boomExtend(boomExtendSpeed);
                extensionMotorIsStopped = false;
            } else if (!extensionMotorIsStopped) {
                //robot.boomExtendStop();
                robot.boomExtend(0);
                extensionMotorIsStopped = true;
            }

            telemetry.addData("Extend motor voltage", robot.boomExtendMotor.getPower());
            telemetry.addData("Extend motor current position", robot.boomExtendMotor.getCurrentPosition());
            telemetry.addData("Extend motor target", robot.boomExtendMotor.getTargetPosition());

            if (gamepad2.a) {
                robot.rightServoRelease();
                telemetry.addData("Right Servo", "release");
            } else if (gamepad2.b) {
                robot.rightServoClamp();
                telemetry.addData("Right Servo", "clamp");
            } else if (gamepad2.x) {
                robot.leftServoRelease();
                telemetry.addData("Left Servo", "release");
            } else if (gamepad2.y) {
                robot.leftServoClamp();
                telemetry.addData("Left Servo", "clamp");
            }

            telemetry.addData("gryo", robot.getRawGyroHeading());

            telemetry.update();
        }
    }

}