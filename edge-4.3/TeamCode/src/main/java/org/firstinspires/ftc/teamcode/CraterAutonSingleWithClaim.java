/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Crater - Single Sample, with Claim", group = "Concept")
//@Disabled
public class CraterAutonSingleWithClaim extends LinearOpMode {
    EdgeBot robot;

    boolean goldSeen = false;
    cubeLocation location = cubeLocation.UNKNOWN;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.initTfod(hardwareMap);
        } else {
            telemetry.addData("Error", "Cannon initialize TFObjectDetector");
        }

        /** Wait for the game to begin */
        telemetry.addData("Quote of the program: ", "I have brought PEACE, JUSTICE, and SECURITY to my NEW EMPIRE!");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (robot.tfod != null) {
                robot.tfod.activate();
            }

            robot.robotLowerAuton();

            robot.liftServoRelease();
            sleep(1000);

            robot.driveForwardForInches(3.25, 0.3, telemetry);
            sleep(200);

            timer.reset();

            while (opModeIsActive() && location == cubeLocation.UNKNOWN && timer.milliseconds() < 500.0) {
                location = robot.detectCube(telemetry);
            }

            robot.driveForwardForInches(3, 0.2, telemetry);
            sleep(200);

            timer.reset();

            while (opModeIsActive() && location == cubeLocation.UNKNOWN && timer.milliseconds() < 500.0) {
                location = robot.detectCube(telemetry);
            }

            if (location == cubeLocation.UNKNOWN) {
                location = cubeLocation.CENTER;
                telemetry.addData(">>", "Defaulting to center");
                telemetry.update();
            }

            if (location == cubeLocation.LEFT) {
                robot.rotateCounterClockwiseGyro(35, 1, telemetry);
                sleep(200);

                robot.driveForwardForInches(25, 0.7, telemetry);

                robot.flipServoUp();

                robot.driveBackwardForInches(8, 0.6, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(55, 1, telemetry);
                sleep(200);

                robot.driveForwardForInches(40, 0.7, telemetry);
                sleep(200);

                robot.driveForwardForSeconds(1750, 0.15, telemetry);

                robot.rotateCounterClockwiseGyro(35, 1,telemetry);
                sleep(200);

                //robot.driveForwardForInches(43, 0.7, telemetry);
                robot.driveToMarkerDistance(0.6, 11.5, telemetry);

                robot.boomRotateAuton();
                robot.flipServoDown();

                robot.driveBackwardForInches(12, 0.6, telemetry);
            } else if (location == cubeLocation.CENTER) {
                /*robot.driveForwardForInches(20, 0.6, telemetry);
                sleep(200);

                robot.flipServoUp();

                robot.driveBackwardForInches(8, 0.6, telemetry);
                sleep(200);

                //robot.driveForwardForInches(21.4, 0.6, telemetry);
                */

                robot.driveForwardForInches(21, 0.6, telemetry);

                robot.flipServoUp();

                robot.driveBackwardForInches(7, 0.6, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(90, 1, telemetry);
                sleep(200);

                robot.driveForwardForInches(49, 0.6, telemetry);
                sleep(200);

                robot.driveForwardForSeconds(1750, 0.15, telemetry);

                robot.rotateCounterClockwiseGyro(35, 1,telemetry);
                sleep(200);

                robot.driveToMarkerDistance(0.7, 7, telemetry);

                robot.boomRotateAuton();
                robot.flipServoDown();

                robot.driveBackwardForInches(12, 0.6, telemetry);
                sleep(200);
            } else if (location == cubeLocation.RIGHT) {
                robot.rotateClockwiseGyro(35, 1, telemetry);
                sleep(200);

                robot.driveForwardForInches(25, 0.6, telemetry);
                sleep(200);

                robot.driveBackwardForInches(10, 0.6, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(125, 1, telemetry);
                sleep(200);

                robot.driveForwardForInches(48, 0.6, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(40, 1,telemetry);
                sleep(200);

                robot.driveForwardForInches(44, 0.6, telemetry);
                sleep(200);

                robot.boomRotateAuton();

                robot.driveBackwardForInches(17, 0.6, telemetry);
            }
        }

        if (opModeIsActive()) {
            robot.boomRotateAuton();
        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }

        while (opModeIsActive()) {
            sleep(20);
        }
    }
}
