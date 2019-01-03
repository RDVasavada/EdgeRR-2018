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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Driver Control")
//@Disabled
public class EdgeTeleop extends LinearOpMode {

    EdgeBot robot;

    // For braking the arm motors
    boolean extensionMotorIsStopped = false;

    boolean armInPreset = false;
    boolean armToLander = false;

    // For the sweeper intake (-1 reverse, 0 stop, 1 forwards)
    int leftSweeperPos = 0;
    int leftSweeperToggle = 1;

    int rightSweeperPos = 0;
    int rightSweeperToggle = 1;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

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
            double rotateSpeed = gamepad1.left_stick_x;

            double liftSpeed = gamepad1.right_trigger - gamepad1.left_trigger;

            robot.tankDrive(driveSpeed, rotateSpeed);

            robot.robotLift(liftSpeed);

            if (gamepad1.a) {
                robot.liftServoClamp();
            } else if (gamepad1.b) {
                robot.liftServoRelease();
            }

            /* *** GamePad Two *** */
            double boomAngleSpeed = gamepad2.right_stick_y * -1;
            double boomRotateSpeed = gamepad2.right_stick_x / 2;
            double boomExtendSpeed = (gamepad2.left_stick_y) * -0.25;

            if (gamepad2.dpad_up) {
                robot.armHomePosition();
                armInPreset = true;
            } else if (gamepad2.dpad_down) {
                robot.armDownPosition();
                armInPreset = true;
            } else if (gamepad2.dpad_right) {
                robot.armGoldPosition();
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

            if (Math.abs(boomRotateSpeed) > 0.05) {
                robot.boomRotate(boomRotateSpeed);
                armInPreset = false;
            }  else if (!armInPreset) {
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
            extendPID.d = 2;
            robot.boomExtendMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, extendPID);

            telemetry.addData("Extend motor p", extendPID.p);
            telemetry.addData("Extend motor i", extendPID.i);
            telemetry.addData("Extend motor d", extendPID.d);
            telemetry.addData("Extend motor tolerance", robot.boomExtendMotor.getTargetPositionTolerance());

            /*
            if (gamepad2.left_bumper) {
                if (!leftBumperPressed) {
                    if (leftSweeperPos == -1) {
                        robot.leftIntakeStop();
                        leftSweeperPos = 0;
                        leftSweeperToggle = 1;
                    } else if (leftSweeperPos == 0) {
                        if (leftSweeperToggle == -1) {
                            robot.leftIntakeReverse();
                            leftSweeperPos = -1;
                        } else if (leftSweeperToggle == 1) {
                            robot.leftIntakeSweep();
                            leftSweeperPos = 1;
                        }
                    } else if (leftSweeperPos == 1) {
                        robot.leftIntakeStop();
                        leftSweeperPos = 0;
                        leftSweeperToggle = -1;
                    }
                }

                leftBumperPressed = true;
            } else {
                leftBumperPressed = false;
            }

            if (gamepad2.right_bumper) {
                if (!rightBumperPressed) {
                    if (rightSweeperPos == -1) {
                        robot.rightIntakeStop();
                        rightSweeperPos = 0;
                        rightSweeperToggle = 1;
                    } else if (rightSweeperPos == 0) {
                        if (rightSweeperToggle == -1) {
                            robot.rightIntakeReverse();
                            rightSweeperPos = -1;
                        } else if (rightSweeperToggle == 1) {
                            robot.rightIntakeSweep();
                            rightSweeperPos = 1;
                        }
                    } else if (rightSweeperPos == 1) {
                        robot.rightIntakeStop();
                        rightSweeperPos = 0;
                        rightSweeperToggle = -1;
                    }
                }

                rightBumperPressed = true;
            } else {
                rightBumperPressed = false;
            }
            */

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

            if (gamepad2.a) {
                robot.setArmHome();
            }

            telemetry.update();
        }
    }

}