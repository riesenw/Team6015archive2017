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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Motor channel:  left claw motor:  "left_claw"
 * Motor channel:  right claw motor: "right_hand"
 */


public class HardwareSummer18Demo

{
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftArm = null;
    //public DcMotor leftClaw = null;
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public DigitalChannel bottomStop;
    public static final int TICKS_PER_DEGREE = 17;
    public static final int ARC_TICKS_PER_DEGREE = 65;
    public static final int DRIVE_TICKS_PER_INCH = 200;
    public static final int MSEC_PER_BLOCK_LEVEL = 4000;

    //public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    static final double INCREMENT = 0.005;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 10;     // period of each cycle
    static final double MAX_PWR = 0.6;     // Maximum power applied to motor
    static final double START_ROLL_POWER = 0.05;

    // Define class members
    double power = 0;
    double fractionDone = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime timer = new ElapsedTime();


    /* Constructor */
    public HardwareSummer18Demo() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftClaw = hwMap.get(Servo.class, "left_claw");
        rightClaw = hwMap.get(Servo.class, "right_claw");
        leftArm = hwMap.get(DcMotor.class, "left_arm");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Try leaving as is...
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to our digitalTouch object.
        bottomStop = hwMap.get(DigitalChannel.class, "bottom_stop");
        // set the digital channel to input.
        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }

    public void go(int inches) {
        int ticks = 0;
        //Straight drive a specified distance by running drive motors together
        //convert degrees to encoder ticks
        ticks = inches * DRIVE_TICKS_PER_INCH;
        power = 0;
        //set targets
        leftDrive.setTargetPosition(ticks);
        rightDrive.setTargetPosition(ticks);
        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);
        //loop until set target reached
        while (leftDrive.isBusy() || rightDrive.isBusy()) {
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pivot(int degrees) {
        int ticks = 0;
        //Pivot turn by running drive motors opposite directions
        //First rezero encoders and make sure the drive motors are in correct mode
        //next convert degrees to encoder ticks
        ticks = degrees * TICKS_PER_DEGREE;
        //set targets and engage motors
        leftDrive.setTargetPosition(ticks);
        rightDrive.setTargetPosition(-ticks);
        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);
        //loop until set targets reached
        while (leftDrive.isBusy() || rightDrive.isBusy()) {
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void raise(int ticks) {
        leftArm.setTargetPosition(ticks);
        leftArm.setPower(0.2);
        while (leftArm.isBusy()) {
        }
        leftArm.setPower(0);
    }

    public void lower(int ticks) {
        //lower arm for specified time unless limit switch at bottom is hit:
        leftArm.setTargetPosition(-ticks);
        leftArm.setPower(0.2);
        while (leftArm.isBusy()) {
        }
        leftArm.setPower(0);
    }


    public void openClaw() {
        //open claw
        rightClaw.setPosition(0);
        leftClaw.setPosition(30);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void closeClaw() {
        //close claw to grab block
        rightClaw.setPosition(30);
        leftClaw.setPosition(0);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void rightWaveClaw() {
        rightClaw.setPosition(30);
        leftClaw.setPosition(30);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void leftWaveClaw() {
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void moveBlock(boolean reverse, int fromLevel, int toLevel) {
        openClaw();
        lower(10000);  // first move to bottom to establish position
        raise(fromLevel * MSEC_PER_BLOCK_LEVEL);// go to grab height
        closeClaw();
        raise(1500); // go up a little more to carry block
        arc("left", -90);
        if (fromLevel > toLevel) {
            lower((fromLevel - toLevel) * MSEC_PER_BLOCK_LEVEL);
        }
        if (toLevel > fromLevel) {
            raise((toLevel - fromLevel) * MSEC_PER_BLOCK_LEVEL);
        }
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        arc("right", 90);
        lower(1500); //lower back down into position
        openClaw();
        openClaw();
    }

    public void returnEmpty() {
        arc("right", -90);
        arc("left", 90);
    }

    public void arc(String direction, int degrees) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = degrees * ARC_TICKS_PER_DEGREE;
        double power = 0;
        double increment = 0.01;
        if (direction == "left") {
            leftDrive.setTargetPosition(ticks);
            rightDrive.setTargetPosition(ticks / 2);
            while (leftDrive.isBusy() || rightDrive.isBusy())
                while (power <= MAX_PWR) {
                    power += increment;
                    leftDrive.setPower(power);
                    rightDrive.setPower(power / 2);
                }
        } else {
            leftDrive.setTargetPosition(ticks / 2);
            rightDrive.setTargetPosition(ticks);
            while (leftDrive.isBusy() || rightDrive.isBusy())
                while (power <= MAX_PWR) {
                    power += increment;
                    leftDrive.setPower(power / 2);
                    rightDrive.setPower(power);
                }
        }
        //loop until set targets reached
        while (leftDrive.isBusy() || rightDrive.isBusy()) {
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean bottomLimitIsPressed() {
        return !bottomStop.getState();
    }

    public void calibrateElevator() {
        // Is bottom limit switch already pressed?
        if (bottomLimitIsPressed()) {
            getOffMyToe(); //Raise off switch, reset encoder, done.
        } else {
            // where do we think the elevator is?
            // Way above zero, close to or possibly below zero
            if (leftArm.getCurrentPosition() > 2000) {  //We can descend to 1000 a little faster as we are not close
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setTargetPosition(2000);
                leftArm.setPower(0.2);
                while (leftArm.isBusy()) {
                    //wait until position 2000 is reached.
                }
            }
            //descend the rest of the way to switch: better go slow.
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftArm.setPower(-0.05);
            while (bottomLimitIsPressed() == false) {
                try {
                    Thread.sleep(10); //wait a moment to allow other stuff like reading switch
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            getOffMyToe(); //when switch is pressed, stop and reset. Done.
        }
    }

    public void getOffMyToe() {
        // This block of code is used when limit switch is pressed.
        // It backs up the elevator until no longer pressed and then sets zero calibration
        //raise elevator until no longer pressed.
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setPower(0.2);
        while (bottomLimitIsPressed()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        // when switch is released, that is your zero point. Reset.
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

