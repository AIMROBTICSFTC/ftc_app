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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="9997Arcade", group="9997")
//@Disabled
public class TestCode9997 extends LinearOpMode {


    Team9997Hardware robot  = new Team9997Hardware();

    @Override
    public void runOpMode() {
        double arm1;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        double clawPosition = 0;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
           /*if (gamepad1.right_trigger > 0){
               robot.arm1.setPosition(100);
           }
              else{
               robot.arm1.setPosition(10);
           }

*/

            robot.arcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);


            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                robot.armPosition += robot.ARM_SPEED;
            else if (gamepad1.y)
                robot.armPosition -= robot.ARM_SPEED;
            if (gamepad1.a){
                clawPosition = 0;
            }
            else if (gamepad1.b) {
                clawPosition = 0.5;

            }else if (gamepad1.y) {

                clawPosition = 1.0;
            }



            telemetry.addData("claw position is ", clawPosition);


            // Move both servos to new position.
            // robot.armPosition  = Range.clip(robot.armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            //   robot.arm1.setPosition(robot.armPosition);



            // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.clawR.setPosition(1.00-clawPosition);
            robot.clawL.setPosition(clawPosition);

            /*
            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;
*/

            // Move both servos to new position.
          //  robot.armPosition  = Range.clip(robot.armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
          //  robot.arm1.setPosition(robot.armPosition);

/*

            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

*/
            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", robot.armPosition);
//            telemetry.addData("claw",  "%.2f", clawPosition);
            //telemetry.addData("left",  "%.2f", left);
            //telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
