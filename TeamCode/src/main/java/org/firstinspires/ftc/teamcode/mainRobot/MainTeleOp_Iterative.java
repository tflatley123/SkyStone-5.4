
package org.firstinspires.ftc.teamcode.mainRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mainRobot.MainRobotHardware;


@TeleOp(name="MainTeleOp Iterative", group="Opmode")
//@Disabled
public class MainTeleOp_Iterative extends OpMode
{
    MainRobotHardware robot = new MainRobotHardware();
    ElapsedTime runtime = new ElapsedTime();




    // runs once (on init)
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initializing");

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }


    // runs repeatedly (after Init, before Play)
    @Override
    public void init_loop()
    {
        // prolly not using this
    }


    // runs once (after play)
    @Override
    public void start()
    {
        runtime.reset();
    }


     // runs repeatedly (after play, before stop)
    @Override
    public void loop()
    {
        // tank drive
        robot.backleftPower = gamepad1.left_stick_y;
        robot.frontleftPower = gamepad1.left_stick_y;
        robot.frontrightPower = gamepad1.right_stick_y;
        robot.backrightPower = gamepad1.right_stick_y;

        // zero the powers to each servo
        robot.zeroServoPower();

        // sideways driving will override the tank drive (only if it is being used though)
        if((gamepad1.dpad_right || gamepad1.dpad_left) || (gamepad1.dpad_up || gamepad1.dpad_down))
        {
            if (gamepad1.dpad_left) // left
            {
                moveForward(0.6);

            } else if (gamepad1.dpad_right) // right
            {
                moveBackward(0.6);
            }

            if (gamepad1.dpad_down) // down
            {
                moveRight(0.7);

            }
            else if (gamepad1.dpad_up) // forward
            {
                moveLeft(0.7);
            }

            if (gamepad1.dpad_right && gamepad1.dpad_up) // up right
            {
                // up left
                robot.backleftPower = -0.8;
                robot.frontleftPower = 0.15;
                robot.frontrightPower = -0.8;
                robot.backrightPower = 0.15;
            }
            else if (gamepad1.dpad_right && gamepad1.dpad_down) // down right
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

            }
            else if (gamepad1.dpad_left && gamepad1.dpad_down) // down left
            {

                // down right
                robot.backleftPower = 0.8;
                robot.frontleftPower = -0.15;
                robot.frontrightPower = 0.8;
                robot.backrightPower = -0.15;
            }
        }


        if(gamepad2.a)
        {
            robot.rightPower = -0.6;

            robot.leftPower = -0.6;
        }
        else if(gamepad2.y)
        {
            robot.rightPower = 0.6;

            robot.leftPower = 0.6;
        }
        if(gamepad2.dpad_down){robot.middlePower = -0.6;}
        else if(gamepad2.dpad_up){robot.middlePower = 0.6;}



        if((gamepad2.dpad_left|| gamepad2.dpad_right) || (gamepad2.x || gamepad2.b) )
        {
            if(gamepad2.dpad_left)
            {
                robot.armPower = -0.65;

            }
            else if(gamepad2.dpad_right)
            {
                robot.armPower = 0.65;
            }

            if(gamepad2.x)
            {
                robot.sledLeftPower = 1;
                robot.sledRightPower = 1;
            }
            else if(gamepad2.b)
            {
                robot.sledLeftPower = -1;
                robot.sledRightPower = -1;
            }

        }

        // 50% power changer
        if(gamepad1.right_trigger > 0)
        {

            robot.backleftPower *= 0.75;
            robot.frontleftPower *= 0.75;
            robot.frontrightPower *= 0.75;
            robot.backrightPower *= 0.75;
        }


        // update the power to each thing on the robot -> should be done every time
        robot.updateMotorPower();
        robot.updateServoPower();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", robot.backleftPower, robot.backrightPower);
    }

    // runs once (when stop is pressed)
    @Override
    public void stop()
    {

    }

    private void moveLeft(double power)
    {
        robot.backleftPower = -power;
        robot.frontleftPower = power;
        robot.frontrightPower = -power;
        robot.backrightPower = power;
    }
    private void moveRight(double power)
    {
        robot.backleftPower = power;
        robot.frontleftPower = -power;
        robot.frontrightPower = power;
        robot.backrightPower = -power;
    }
    private void moveForward(double power)
    {
        robot.backleftPower = power;
        robot.frontleftPower = power;
        robot.frontrightPower = power;
        robot.backrightPower = power;
    }
    private void moveBackward(double power)
    {
        robot.backleftPower = -power;
        robot.frontleftPower = -power;
        robot.frontrightPower = -power;
        robot.backrightPower = -power;
    }



}

/*
TO DO:

up and down on d- pad for all directions        DONE
50 % speed for left trigger                     DONE

 */