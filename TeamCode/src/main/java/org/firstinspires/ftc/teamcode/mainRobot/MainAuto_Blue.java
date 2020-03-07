
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

package org.firstinspires.ftc.teamcode.mainRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.TFOD;

@Autonomous(name="Main Auto Blue", group="Autonomous")
@Disabled
public class MainAuto_Blue extends LinearOpMode {

    private int side = -1;
    private TFOD detector = new TFOD();
    private MainRobotHardware robot = new MainRobotHardware();

    private boolean isRun(ElapsedTime x)
    {
        return x.seconds() < 26;
    }

    private void allignWithSkystone(ElapsedTime x)
    {

        boolean isAlligned = false;

        while (!isAlligned && isRun(x))
        {
            keepStraight();
            detector.updateDetectedObjects(); // scan for objects
            if(detector.recognizedObjects != null)
            {
                for (Recognition object : detector.recognizedObjects)// go through list of objects
                {
                    if (object.getLabel().equals("Skystone")) // if a detected object is a skystone
                    {

                        // make initial alignment (stage = 0)
                        double movePower = 0;// detector.moveTowardSkystone(object, side); // get power to do (see TFOD)

                        robot.moveFB(-movePower);
                        sleep(50);
                        robot.moveFB(0);

                        if(movePower > 0) // if its needs to move right
                        {
                            telemetry.addData("go:", "right");
                        }
                        else if (movePower < 0) // if it needs to move left
                        {
                            telemetry.addData("go:", "left");
                        }
                        else
                        {
                            telemetry.addData("go: ", "forward");
                            isAlligned = true;
                            break;
                        }

                        telemetry.update();
                    }
                    /* moves to the wall for to scan others
                      ( no skystone in view )
                      is not up against the wall */
                    else
                    {
                        robot.moveFB(-.5);
                        sleep(100);
                        robot.moveFB(0);
                        sleep(450);

                        telemetry.addData("Move: ", "researching");
                        telemetry.update();
                    }
                    sleep(50);
                }
            }
            else
            {
                robot.moveFB(-.4);
                sleep(100);
                robot.moveFB(0);
                sleep(450);

                telemetry.addData("Move: ", "researching");
                telemetry.update();
            }
        }
        robot.moveFB(0); // stop moving
    }

    @Override
    public void runOpMode()
    {

        telemetry.addData("button ", "pressed");
        robot.init(hardwareMap);

        if (detector.initialize(hardwareMap))
        {
            telemetry.addData("DETECTOR", "has initialized");
        } else {
            telemetry.addData("DETECTOR", "Has Failed to initialize");
            return;
        }
        telemetry.update();
        detector.updateDetectedObjects();
        waitForStart();

        while (opModeIsActive())
        {
            ElapsedTime runtime = new ElapsedTime();

            while(robot.blocksideDistance.getDistance(DistanceUnit.CM) > 32 && isRun(runtime))
            {
                keepStraight();
                robot.moveLR(0.4);
            }
            allignWithSkystone(runtime);
            robot.moveFB(-.4);
            sleep(150);
            robot.moveFB(0);

            while(robot.blocksideDistance.getDistance(DistanceUnit.CM) > 6.5 && isRun(runtime)) // move forward to grab block
            {
                robot.moveLR(0.5);
            }
            robot.moveLR(0);

            if(isRun(runtime))
            {
                robot.armServo.setPower(1);
                sleep(500);                     // 3. to put arm down

                robot.middleServo.setPower(1);
                sleep(150);                  // 4.  time to lift for
                robot.middleServo.setPower(0);

                robot.moveLR(-.6);
                sleep(850);                 //5. move backward
                robot.moveLR(0);
            }
            while(robot.backDistance.getDistance(DistanceUnit.CM) > 100 && isRun(runtime)) //6. move to otherside
            {
                keepStraight();
                robot.moveFB(.8);
            }
            if(isRun(runtime))
            {
                robot.moveFB(0);
                robot.armServo.setPower(-1);              //7. release block
                sleep(500);
                robot.middleServo.setPower(-1);

                robot.moveLR(0.6);
                sleep(850);                     //8. move forward
                robot.moveLR(0);
                robot.armServo.setPower(0);
                robot.middleServo.setPower(0);

                robot.moveLR(-0.6);
                sleep(850);                     // 9. move backwarde
                robot.moveLR(0);
            }
            /*
            robot.moveFB(1);
            sleep(1000);
            robot.moveLR(0);


            //REPEAT FIRST COLLECTION
            allignWithSkystone(0);
            robot.moveLR(0);
            while(robot.blocksideDistance.getDistance(DistanceUnit.CM) > 30)
            {
                keepStraight();
                robot.moveLR(0.4);
            }

            allignWithSkystone(1);
            while(robot.blocksideDistance.getDistance(DistanceUnit.CM) > 6.5) // move forward to grab block
            {
                robot.moveLR(0.5);
            }
            robot.moveLR(0);

            robot.armServo.setPower(1);
            sleep(500);                     // 3. to put arm down

            robot.middleServo.setPower(1);
            sleep(150);                  // 4.  time to lift for
            robot.middleServo.setPower(0);

            robot.moveLR(-.6);
            sleep(850);                 //5. move backward
            robot.moveLR(0);


            while(robot.frontDistance.getDistance(DistanceUnit.CM) > 100) //6. move to otherside
            {
                keepStraight();
                robot.moveFB(-.8);
            }
            robot.moveFB(0);
            robot.armServo.setPower(-1);              //7. release block
            sleep(200);

            robot.middleServo.setPower(-1);
            sleep(140);                  // 4.  drop linear slide
            robot.middleServo.setPower(0);


            robot.moveLR(0.6);
            sleep(850);                     //8. move forward
            robot.moveLR(0);
            robot.armServo.setPower(0);

            robot.moveLR(-0.6);
            sleep(300);                     // 9. move backward
            robot.moveLR(0);
            */
            while(robot.lineColor.blue() < 1000 && isRun(runtime)) //6. move to otherside
            {
                keepStraight();
                robot.moveFB(-.4);
            }
            robot.moveFB(0);

            break;

        }
        detector.shutDown();

    }

    public void keepStraight()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle > 0.8) {
            // turn right
            robot.turn(0.3);
            sleep(30);
            robot.turn(0);
        } else if (angles.firstAngle < -.8) {
            // turn left
            robot.turn(-0.3);
            sleep(30);
            robot.turn(0);
        }
    }
}