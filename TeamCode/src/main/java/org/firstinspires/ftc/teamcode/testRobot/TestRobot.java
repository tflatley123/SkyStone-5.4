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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an opmode.
 *
 * This is class for hardware of the robot.
 */
public class TestRobot
{
    // ticks per rev on the 60:1 motors
    private final int andymark60to1Ticks = 1680;
    /* Public OpMode members. */
    public DcMotor  frontleft   = null;
    public DcMotor  frontright  = null;
    public DcMotor  backleft    = null;
    public DcMotor  backright   = null;
    public DcMotor  linearSlide = null;

    public CRServo armServo = null;
    public CRServo sledRight, sledLeft;

    public DistanceSensor blocksideDistance, wallsideDistance;
    public ColorSensor blockColorRight, blockColorLeft, lineColor;
    public BNO055IMU imu; // gyro and accelorometer

    public double frontleftPower, frontrightPower, backleftPower, backrightPower;  // motor powers

    // used to map devices to the robot
    public HardwareMap hwMap = null;
    public OpMode opMode;


    public TestRobot(OpMode theOperationMode) {
        opMode = theOperationMode;
        opMode.telemetry.addLine("constructed");
        opMode.telemetry.update();
    }


    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode.telemetry.addLine("begining to map the imu");
        opMode.telemetry.update();
        imu   = hwMap.get(BNO055IMU.class, "imu");
        opMode.telemetry.addLine("mapped the imu");
        opMode.telemetry.update();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        opMode.telemetry.addLine("inited the imu");
        opMode.telemetry.update();

        // Define and Initialize Motors
        frontleft   = hwMap.get(DcMotor.class, "frontleft");
        frontright  = hwMap.get(DcMotor.class, "frontright");
        backleft    = hwMap.get(DcMotor.class, "backleft");
        backright   = hwMap.get(DcMotor.class, "backright");
        linearSlide = hwMap.get(DcMotor.class, "linearslide");
        opMode.telemetry.addLine("inited the motors");
        opMode.telemetry.update();

        armServo = hwMap.get(CRServo.class, "arm");
        sledLeft = hwMap.get(CRServo.class, "leftservo");
        sledRight = hwMap.get(CRServo.class, "rightservo");

        // SET THE DIRECTIONS
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        // MAKE ALL MOTORS HAVE 0 POWER
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        // Set all motors to run with the encoders.
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set up the i2c devices
        blocksideDistance = hwMap.get(DistanceSensor.class, "blocksidedistance");
        wallsideDistance = hwMap.get(DistanceSensor.class, "distancetowall");

        blockColorLeft = hwMap.get(ColorSensor.class, "blockcolorleft");
        blockColorRight = hwMap.get(ColorSensor.class, "blockcolorright");
        lineColor = hwMap.get(ColorSensor.class, "undercolor");
    }

    public void updateMotorPower()
    {
        // updates the power of motors
        frontright.setPower(frontrightPower);
        backright.setPower(backrightPower);
        frontleft.setPower(frontleftPower);
        backleft.setPower(backleftPower);
    }

    public void zeroMotorPower()
    {
        // just test one of the motors to see if it is in encoder mode
        if(frontleft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
            resetEncoder(); // if it is in ENCODER mode,  it will also reset the encoder

        frontrightPower = 0;
        frontleftPower = 0;
        backrightPower = 0;
        backleftPower = 0;
        updateMotorPower();
    }

    public void moveFB(double power, int ticks)
    {
        // positive input makes robot go forward
        // negative input makes robot go backward

        frontrightPower = power;
        frontleftPower = power;
        backrightPower = power;
        backleftPower = power;

        resetEncoder();
        backleft.setTargetPosition(ticks);
        frontleft.setTargetPosition(ticks);
        frontright.setTargetPosition(ticks);
        backright.setTargetPosition(ticks);
        runToPosition();
        updateMotorPower();
    }

    public void moveFB(double power)
    {
        // positive input makes robot go forward
        // negative input makes robot go backward
        frontrightPower = power;
        frontleftPower = power;
        backrightPower = power;
        backleftPower = power;

        updateMotorPower();
    }

    public void moveLR(double power, int ticks)
    {
        // positive power makes the robot strafes left
        // negative power makes the robot strafes right

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleftPower = -power;
        frontleftPower = power;
        frontrightPower = -power;
        backrightPower = power;

        resetEncoder();

        backleft.setTargetPosition(-ticks);
        frontleft.setTargetPosition(ticks);
        frontright.setTargetPosition(-ticks);
        backright.setTargetPosition(ticks);
        runToPosition();
        updateMotorPower();
    }

    public void moveLR(double power)
    {
        // positive power makes the robot strafes left
        // negative power makes the robot strafes right
        backleftPower   = -power;
        frontleftPower  = power;
        frontrightPower = -power;
        backrightPower  = power;
        updateMotorPower();
    }

    public void rotate(double power, int ticks)
    {
        // if power is positive the robot rotates left
        // if power is positive the robot rotates right

        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleftPower   = power;
        frontleftPower  = power;
        frontrightPower = power;
        backrightPower  = power;

        resetEncoder();
        backleft.setTargetPosition(-ticks);
        frontleft.setTargetPosition(-ticks);
        frontright.setTargetPosition(ticks);
        backright.setTargetPosition(ticks);
        runToPosition();
        updateMotorPower();
    }

    private void resetEncoder()
    {
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void runToPosition()
    {
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runWithoutEncoders()
    {
        resetEncoder();
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public boolean motorsAreBusy()
    {
        return( frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy());
    }

 }

