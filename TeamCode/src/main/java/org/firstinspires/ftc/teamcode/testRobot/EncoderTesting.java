/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.testRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="EncoderTesting", group="TeleOp")
//@Disabled
public class EncoderTesting extends LinearOpMode
{
    private TestRobot robot = new TestRobot(this);

    @Override
    public void runOpMode()
    {
        int moveTicks = 0;
        int addedTicks = 0;


        robot.init(hardwareMap);
        waitForStart();


        while (opModeIsActive())
        {
            if(gamepad1.a) {
                robot.moveFB(.5, moveTicks);
                addedTicks += moveTicks;
            }
            else if(gamepad1.y)
                addedTicks = 0;
            if(gamepad1.b)
            {
                moveTicks += 5;
            }
            else if(gamepad1.x)
            {
                moveTicks -= 5;
            }


            telemetry.addData("Distance to Block: ", robot.blocksideDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Ticks", moveTicks);
            telemetry.addLine("RIGHT");
            telemetry.addData("R", robot.blockColorRight.red());
            telemetry.addData("B", robot.blockColorRight.blue());
            telemetry.addData("G", robot.blockColorRight.green());
            telemetry.addLine("LEFT");
            telemetry.addData("R", robot.blockColorLeft.red());
            telemetry.addData("B", robot.blockColorLeft.blue());
            telemetry.addData("G", robot.blockColorLeft.green());
            telemetry.addLine("UNDER");
            telemetry.addData("R", robot.lineColor.red());
            telemetry.addData("B", robot.lineColor.blue());
            telemetry.addData("G", robot.lineColor.green());

            telemetry.update();

        }

    }
    private void keepStraight()
    {
        boolean isAlligned = false;

        while(isAlligned) {
            if(!robot.motorsAreBusy()) {
                Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (angles.firstAngle > 0.5) {
                    // turn right
                    robot.rotate(0.6, 35);
                } else if (angles.firstAngle < -.4) {
                    // turn left
                    robot.rotate(0.3, -35);
                    sleep(30);
                } else
                {
                    isAlligned = true;
                }
            }
        }

    }
}
