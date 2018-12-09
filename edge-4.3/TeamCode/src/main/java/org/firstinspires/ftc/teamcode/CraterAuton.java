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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auton Test - Distance Sensors")
//@Disabled
public class CraterAuton extends LinearOpMode {

    EdgeBot robot;

    @Override
    public void runOpMode() {
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        //robot.robotLowerAuton();

        while (opModeIsActive()) {
            robot.driveForwards(0.05);

            double topSensorDistance = robot.getTopSensorDistance(DistanceUnit.INCH);
            double bottomSensorDistance = robot.getBottomSensorDistance(DistanceUnit.INCH);

            boolean triggeredTop = false;
            boolean triggeredBottom = false;

            if (topSensorDistance < 10) {
                triggeredTop = true;
            }

            if (bottomSensorDistance < 10) {
                triggeredBottom = true;
            }

            if (triggeredBottom && triggeredTop) {
                if (Math.abs(topSensorDistance - bottomSensorDistance) < 2) {
                    telemetry.addData("Object detected", "sphere");
                }
            } else if (triggeredBottom) {
                if (topSensorDistance - bottomSensorDistance < 6) {
                    telemetry.addData("Object detected", "cube");
                    robot.stopDriveMotors();
                }
            }

            telemetry.addData("top sensor", robot.getTopSensorDistance(DistanceUnit.INCH));
            telemetry.addData("bottom sensor", robot.getBottomSensorDistance(DistanceUnit.INCH));

            telemetry.update();
        }
    }

}