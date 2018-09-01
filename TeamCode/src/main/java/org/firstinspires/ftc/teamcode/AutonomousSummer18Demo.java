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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Here is my attempt to have the robot stack blocks one at a time.
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "AutonomousSummer18Demo", group = "Linear Opmode")
//@Disabled
public class AutonomousSummer18Demo extends LinearOpMode {
    // Declare OpMode members.
    HardwareSummer18Demo robot = new HardwareSummer18Demo();
    private ElapsedTime runtime = new ElapsedTime();
    public static final int TICKS_CLEARANCE_HEIGHT = 1000;
    public static final int TICKS_PER_BLOCK_LEVEL = 7800;
    // Define class members

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //   telemetry.addData("Status", "Initialized");
        //   telemetry.update();
        robot.calibrateElevator();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.openClaw();

        stackBlockAndReturn(3,1);
        stackBlockAndReturn(2,2);
        stackBlockAndReturn(1,3);
    }

    public int convertLevelToTicks(int level) {
        int ticks = (level - 1) * TICKS_PER_BLOCK_LEVEL;
        return ticks;
    }

    public void stackBlockAndReturn(int fromLevel, int toLevel) {
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int grabHeight = convertLevelToTicks(fromLevel);
        robot.leftArm.setTargetPosition(grabHeight+100);
        robot.leftArm.setPower(0.2);
        while (robot.leftArm.isBusy()){
            if (robot.bottomLimitIsPressed()){
                robot.getOffMyToe();
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            }
        }
        robot.closeClaw();  //grab block at chosen level.
        robot.leftArm.setTargetPosition(grabHeight+ TICKS_CLEARANCE_HEIGHT); // lift clear of floor or block below
        robot.leftArm.setPower(0.4);
        while (robot.leftArm.isBusy()) {
        }
        robot.leftArm.setPower(0);
        robot.arc("left", -90); //Back up and turn
        sleep(60);
        robot.leftArm.setTargetPosition(TICKS_CLEARANCE_HEIGHT + convertLevelToTicks(toLevel)); //raise to correct level.
        robot.leftArm.setPower(0.4);
        while (robot.leftArm.isBusy()) {
        }
        robot.leftArm.setPower(0);
        robot.arc("right", 90); //Forward and turn to destination
        robot.leftArm.setTargetPosition(convertLevelToTicks(toLevel)); //lower block to dropoff point
        robot.leftArm.setPower(0.1);
        while (robot.leftArm.isBusy()) {
        }
        robot.leftArm.setPower(0);
        robot.openClaw(); //release  block
        robot.arc("right", -90); // return to original position
        sleep(60);
        //robot.leftArm.setTargetPosition(100); //return to near-zero height
        //robot.leftArm.setPower(0.2);
        //while (robot.leftArm.isBusy()) {
        //    if (robot.bottomLimitIsPressed()) {
        //        robot.getOffMyToe();
        //        break;
        //    }
        //}
        //robot.leftArm.setPower(0);
        if (toLevel == 3){
            salute();
        }
        else {
            robot.arc("left", 90);
        }
    }

    public void salute(){
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setTargetPosition(6000);
        robot.leftDrive.setTargetPosition(-6000);
        robot.rightDrive.setPower(0.7);
        robot.leftDrive.setPower(0.7);
        while(robot.rightDrive.isBusy() || robot.leftDrive.isBusy()){
        }
        robot.closeClaw();
        robot.openClaw();
        robot.closeClaw();
        robot.openClaw();
        robot.closeClaw();
        robot.openClaw();

    }

}


