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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Driver Control (With Arm Deploy)")
//@Disabled
public class ArmDeployTest extends LinearOpMode {

    EdgeBot robot;

    // For braking the arm motors
    boolean extensionMotorIsStopped = false;

    boolean armInPreset = false;

    @Override
    public void runOpMode() {
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        robot.setArmMotorsResetEncoders();

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            /* *** GamePad One *** */
            double driveSpeed = gamepad1.left_stick_y;
            double rotateSpeed = gamepad1.right_stick_x;

            double deploySpeed = gamepad1.right_trigger - gamepad1.left_trigger;

            robot.tankDrive(driveSpeed, rotateSpeed);

            robot.runDeployMotor(deploySpeed);

            telemetry.addData("Deploy motor encoder", robot.deploymentMotor.getCurrentPosition());
            telemetry.addData("Deploy motor encoder min", robot.initialDeployCount - 600);
            telemetry.addData("Deploy motor encoder max", robot.initialDeployCount);

            telemetry.addData("Left drive encoder", robot.leftDriveMotor.getCurrentPosition());
            telemetry.addData("Right drive encoder", robot.rightDriveMotor.getCurrentPosition());
            telemetry.addData("Lift count", robot.liftMotor.getCurrentPosition());

            if (gamepad1.a) {
                robot.liftServoClamp();
            } else if (gamepad1.b) {
                robot.liftServoRelease();
            } else if (gamepad1.x) {
                robot.flipServoDown();
            } else if (gamepad1.y) {
                robot.flipServoUp();
            }

            /* *** GamePad Two *** */
            double boomAngleSpeed = gamepad2.right_stick_y * -1;
            double boomExtendSpeed = (gamepad2.left_stick_y) * -0.25;
            double boomRotateSpeed = 0;

            if (Math.abs(gamepad2.left_stick_x) > 0) {
                boomRotateSpeed = gamepad2.left_stick_x * 0.4;
            } else if (Math.abs(gamepad2.right_stick_x) > 0) {
                boomRotateSpeed = gamepad2.right_stick_x;
            }

            if (gamepad2.dpad_up) {
                robot.armHomePosition();
                armInPreset = true;
            } else if (gamepad2.dpad_left) {
                robot.armSilverPosition();
                armInPreset = true;
            } else if (gamepad2.dpad_right) {
                robot.armGoldPosition(telemetry);
                armInPreset = true;
            }

            if (Math.abs(boomAngleSpeed) > 0.05) {
                robot.boomAngle(boomAngleSpeed);
                armInPreset = false;
            } else if (!armInPreset) {
                robot.boomAngle(0);
            }

            telemetry.addData("Angle motor count", robot.boomAngleMotor.getCurrentPosition());
            telemetry.addData("Extend motor count", robot.boomExtendMotor.getCurrentPosition());
            telemetry.addData("Extend motor target", robot.boomExtendMotor.getTargetPosition());
            telemetry.addData("Rotate motor count", robot.boomRotateMotor.getCurrentPosition());

            if (Math.abs(boomRotateSpeed) > 0) {
                robot.boomRotate(boomRotateSpeed);
                armInPreset = false;
            } else if (!armInPreset) {
                robot.boomRotate(0);
            }

            if (boomExtendSpeed != 0) {
                robot.boomExtend(boomExtendSpeed);
                extensionMotorIsStopped = false;
                armInPreset = false;
            } else if (!extensionMotorIsStopped && !armInPreset) {
                robot.boomExtendStop();
                extensionMotorIsStopped = true;
            }

            PIDFCoefficients extendPID = robot.boomExtendMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            extendPID.p = 10;
            extendPID.i = 0;
            extendPID.d = 4;
            robot.boomExtendMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, extendPID);
            robot.boomExtendMotor.setTargetPositionTolerance(5);

            telemetry.addData("Extend motor p", extendPID.p);
            telemetry.addData("Extend motor i", extendPID.i);
            telemetry.addData("Extend motor d", extendPID.d);
            telemetry.addData("Extend motor tolerance", robot.boomExtendMotor.getTargetPositionTolerance());

            if (gamepad2.left_trigger > 0) {
                robot.leftIntakeReverse();
            } else if (gamepad2.left_bumper) {
                robot.leftIntakeSweep();
            } else if (gamepad2.x) {
                robot.leftIntakeStop();
            }

            if (gamepad2.right_trigger > 0) {
                robot.rightIntakeReverse();
            } else if (gamepad2.right_bumper) {
                robot.rightIntakeSweep();
            } else if (gamepad2.b) {
                robot.rightIntakeStop();
            }

            if (gamepad2.y) {
                robot.leftIntakeStop();
                robot.rightIntakeStop();
            }

            if (gamepad2.a) {
                robot.setArmHome();
            }

            if (gamepad2.x) {
                robot.setArmSilver();
            }

            if (gamepad2.b) {
                robot.setArmGold();
            }

            telemetry.update();
        }
    }

}