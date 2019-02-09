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

@Autonomous(name = "Depot - Single Sample, with claim and park", group = "Concept")
//@Disabled
public class DepotAutonSingle extends LinearOpMode {
    EdgeBot robot;

    boolean goldSeen = false;
    int goldPos = 0; // -1 = left, 0 = middle, 1 = right

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Error", "Cannon initialize TFObjectDetector");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to begin");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            /*robot.robotLowerAuton();

            robot.liftServoRelease();
            sleep(1000);

            robot.driveBackwardForInches(100, 0.2, telemetry);
            sleep(200);*/

            //Initial forward drives
            robot.driveForwardForSteps(200, 0.3, telemetry);
            sleep(200);

            timer.reset();

            visionLoop: while (opModeIsActive() && timer.milliseconds() < 5000) {
                telemetry.addData("Time", timer.milliseconds() / 1000.0);
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
                              goldSeen = true;
                              goldX = recognition.getRight();
                              telemetry.addData("Gold right", goldX);
                          } else if (recognition.getLabel() == Constants.Vision.LABEL_SILVER_MINERAL) {
                              silverX = recognition.getRight();
                              silverCount++;
                          }
                        }

                        if (!goldSeen) {
                            if (silverCount == 2) {
                                goldPos = 1;
                                telemetry.addData("Gold pos", goldPos);
                                break visionLoop;
                            }
                        } else {
                            if (silverCount == 1) {
                                if (goldX > silverX) {
                                    goldPos = -1;
                                    telemetry.addData("Gold pos", goldPos);
                                    break visionLoop;
                                } else {
                                    goldPos = 0;
                                    telemetry.addData("Gold pos", goldPos);
                                    break  visionLoop;
                                }
                            }
                        }
                      }
                    }
                }
                telemetry.update();
            }

            if (goldPos == -1) { // Left
                robot.driveForwardForSteps(225, 0.2, telemetry);

                robot.rotateCounterClockwiseGyro(40, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(1350, 0.2, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(80, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(1475, 0.2, telemetry);
                sleep(200);

                robot.flipServoUp();
                sleep(200);

                robot.driveBackwardForSteps(275, 0.2, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(80, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(2100, 0.25, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(15, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(800, 0.25, telemetry);
                sleep(200);


            } else if (goldPos == 0) { // Center
                robot.driveForwardForSteps(2200, 0.3, telemetry);
                sleep(200);

                robot.flipServoUp(); //deposits
                sleep(200);

                robot.driveBackwardForSteps(400, 0.2, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(45, 0.8, telemetry);
                sleep(200);

                robot.driveBackwardForSteps(1900, 0.25, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(37, 0.8, telemetry);
                sleep(200);

                robot.driveBackwardForSteps(1300, 0.3, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(75, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(600, 0.8, telemetry);
                sleep(200);
            } else if (goldPos == 1) { // Right
                robot.driveForwardForSteps(240, 0.3, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(40, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(1300, 0.2, telemetry);
                sleep(200);

                robot.driveBackwardForSteps(200, 0.2, telemetry);
                sleep(200);

                robot.rotateCounterClockwiseGyro(85, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(1450, 0.25, telemetry);
                sleep(200);

                robot.flipServoUp(); //drop marker
                sleep(200);

                robot.driveBackwardForSteps(2280, 0.25, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(35, 0.8, telemetry);
                sleep(200);

                robot.driveBackwardForSteps(2300, 0.3, telemetry);
                sleep(200);

                robot.rotateClockwiseGyro(73, 0.8, telemetry);
                sleep(200);

                robot.driveForwardForSteps(400, 0.3, telemetry);
                sleep(200);

                robot.driveBackwardForSteps(300, 0.3, telemetry);
                sleep(200);

                robot.driveForwardForSteps(500, 0.4, telemetry);
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        while (opModeIsActive()) {
            sleep(20);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.Vision.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.Vision.TFOD_MODEL_ASSET, Constants.Vision.LABEL_GOLD_MINERAL, Constants.Vision.LABEL_SILVER_MINERAL);
    }
}
