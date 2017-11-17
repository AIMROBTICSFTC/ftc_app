package org.firstinspires.ftc.teamcode;/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
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
//if (reverse == 1) {
//robot.arcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);
//} else if (reverse == 0) {
//robot.arcadeDrive(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
//}

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Offical Code of AIM Acedemy Robotics Team

    Do not copy any of code unless allowed told to

    This code is for Tank Drive only
    This code also include lift, claw, and flipper

 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ArcadeDrive", group = "9997")
public class TeleOp9997Arcade extends LinearOpMode {

    //Implementing
    private double TOP = 0.9;
    private double BOT = 0.0;
    private double Flip_DELTA = 0.01;
    private double FLIP_POS;

    //Implementing the hardware map
    Team9997Hardware robot = new Team9997Hardware();

    @Override
    public void runOpMode() {

        //Setting variables needed
        double left;
        double right;
        double arm1;
        double lift;
        double ext;
        double clawPosition = 0;
        double hold = 0;
        double reverse = 0;


        //Initialize the hardware variables
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Runs for the entire match (until the drives presses stop)
        while (opModeIsActive()) {

            ext = -gamepad2.right_stick_y;

            robot.extMotor.setPower(ext * Math.abs(ext));

            // If "A" is pressed robot reverses
            // If "B" is pressed robot returns to normal word
            if (gamepad1.a) {
                reverse = 0;
            } else if (gamepad1.b) {
                reverse = 1;
            }

            if (reverse == 0) {
            robot.arcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);
            } else if (reverse == 1) {
            robot.arcadeDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            }

            //Relic Claw Positioning
            if (gamepad2.left_bumper) {

                hold = 0.0;
            } else if (gamepad2.right_bumper) {

                hold = 1.0;
            }

            if(gamepad1.right_bumper){
            robot.leftMotor.setPower(-1.0);
            robot.rightMotor.setPower(1.0);
            } else if(gamepad1.left_bumper){
                robot.leftMotor.setPower(1.0);
                robot.rightMotor.setPower(-1.0);
            }
            //Setting Relic Claw Positioning
            robot.grab.setPosition(hold);

            //Front Claw Positioning

            //Open
            if (gamepad2.y) {
                clawPosition = 1.0;

            }
            //Half Open
            else if (gamepad2.b) {
                clawPosition = 0.55; // change this value  to be closed a little more
            }
            //Closed
            else if (gamepad2.a) {
                clawPosition = 0.0;
            }

            //Setting Positioning for Claw
            robot.clawR.setPosition(1.00 - clawPosition);
            robot.clawL.setPosition(clawPosition);

            //Setting Lift Limit
            if (!robot.bottomLimit.getState()) {

                // Lift Positioning
                if (gamepad2.left_stick_y > 0) {
                    lift = 0;
                } else {
                    lift = -gamepad2.left_stick_y;
                }
            } else {
                lift = -gamepad2.left_stick_y;
            }

            robot.liftMotor.setPower(lift * Math.abs(lift));

            //Flipper back or forward
            if (gamepad2.dpad_up) {
                FLIP_POS = TOP;
                robot.flipper.setPosition(FLIP_POS);
            } else if (gamepad2.dpad_down) {
//                FLIP_POS = BOT;///
//                robot.flipper.setPos/qition(FLIP_POS);
            }
            //Adjust flipper positon

            if (gamepad2.dpad_left) {
                FLIP_POS =+ Flip_DELTA;
                robot.flipper.setPosition(FLIP_POS);
            } else if (gamepad2.dpad_right) {
                FLIP_POS =- Flip_DELTA;
                //FLIP_POS = Range.clip(FLIP_POS, 0.0,1.0);
                robot.flipper.setPosition(FLIP_POS);

            }
        }
    }
}
