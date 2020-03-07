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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.TFOD;

@TeleOp(name="Testing", group="Opmode")
//@Disabled
public class TestTele extends OpMode {
    private TestRobot robot = new TestRobot(this);

    private StonePosition stonePosition;

    // runs once (on init)
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.runWithoutEncoders();
    }


    // runs repeatedly (after Init, before Play)
    @Override
    public void init_loop() {
        // prolly not using this
    }


    // runs once (after play)
    @Override
    public void start() {

    }


    // runs repeatedly (after play, before stop)
    @Override
    public void loop() {
        /*
            SHIFTING
            AND
            DRIFTING
             */
        robot.backleftPower = gamepad1.left_stick_y;
        robot.frontleftPower = gamepad1.left_stick_y;
        robot.frontrightPower = gamepad1.right_stick_y;
        robot.backrightPower = gamepad1.right_stick_y;

        if ((gamepad1.dpad_right || gamepad1.dpad_left) || (gamepad1.dpad_up || gamepad1.dpad_down)) {
            if (gamepad1.dpad_left) // left
            {
                robot.moveFB(0.6);

            } else if (gamepad1.dpad_right) // right
            {
                // move backwards
                robot.moveFB(-0.6);
            }

            if (gamepad1.dpad_down) // down
            {
                // move right
                robot.moveLR(-0.7);
            } else if (gamepad1.dpad_up) // forward
            {
                // move left
                robot.moveLR(0.7);
            }

            if (gamepad1.dpad_right && gamepad1.dpad_up) // up right
            {
                // up left
                robot.backleftPower = -0.8;
                robot.frontleftPower = 0.15;
                robot.frontrightPower = -0.8;
                robot.backrightPower = 0.15;
            } else if (gamepad1.dpad_right && gamepad1.dpad_down) // down right
            {
                // up right
                robot.backleftPower = 0.15;
                robot.frontleftPower = -0.8;
                robot.frontrightPower = 0.15;
                robot.backrightPower = -0.8;

            } else if (gamepad1.dpad_left && gamepad1.dpad_up) // up left
            {
                // down left
                robot.backleftPower = -0.15;
                robot.frontleftPower = 0.8;
                robot.frontrightPower = -0.15;
                robot.backrightPower = 0.8;

            } else if (gamepad1.dpad_left && gamepad1.dpad_down) // down left
            {
                // down right
                robot.backleftPower = 0.8;
                robot.frontleftPower = -0.15;
                robot.frontrightPower = 0.8;
                robot.backrightPower = -0.15;
            }
        }


            /*
            SLED
            CONTROLL
             */

        if (gamepad1.left_bumper) {
            robot.sledLeft.setPower(-1);
            robot.sledRight.setPower(1);
        } else if (gamepad1.right_bumper) {
            robot.sledLeft.setPower(1);
            robot.sledRight.setPower(-1);
        } else {
            robot.sledLeft.setPower(0);
            robot.sledRight.setPower(0);
        }

            /*
            LINAER
            CONTROOOLLL
             */
        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            if (gamepad2.dpad_down) {
                if (gamepad2.right_bumper)
                    robot.linearSlide.setPower(.5);
                else
                    robot.linearSlide.setPower(1);
            } else if (gamepad2.dpad_up) {
                if (gamepad2.right_bumper)
                    robot.linearSlide.setPower(-.5);
                else
                    robot.linearSlide.setPower(-1);
            }
        } else {
            robot.linearSlide.setPower(0);
        }

        if (gamepad2.x || gamepad2.b) {
            if (gamepad2.x)
                // - power does
                robot.armServo.setPower(-1);
            else if (gamepad2.b)
                // + power does
                robot.armServo.setPower(1);
        } else
            robot.armServo.setPower(0);

        robot.updateMotorPower();
    }


    // runs once (when stop is pressed)
    @Override
    public void stop() {

    }
}

