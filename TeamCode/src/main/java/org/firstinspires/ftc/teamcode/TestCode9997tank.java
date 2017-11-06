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
 * gamepad
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

@TeleOp(name="9997TankDrive", group="9997")
//@Disabled
public class TestCode9997tank extends LinearOpMode {

    private double TOP = 0.9;
    private double MID = 0.5;
    private double BOT = 0.0;
    private double Flip_DELTA = 0.01;
    private double FLIP_POS;
    private double FLIPTEMP;
    Team9997Hardware robot  = new Team9997Hardware();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double arm1;
        double lift;
        double ext;
        double clawPosition = 0;
        double hold = 0;
        double flipPosition = 0;
        double flip;
        double reverse = 1;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
         /*  if (gamepad1.right_trigger > 0){
               robot.arm1.setPosition(100);
           }
              else{
               robot.arm1.setPosition(10);
           }
*/
            ext = -gamepad2.right_stick_y;

            robot.extMotor.setPower(ext * Math.abs(ext));

            if (gamepad1.a) {
                reverse = 1;
} else if(gamepad1.b){
                 reverse = 0;
}
//think about a reverse switch


                if (reverse == 1) {
                    right = -gamepad1.right_stick_y;
                    left = gamepad1.left_stick_y;

                    robot.leftMotor.setPower(left * Math.abs(left));
                    robot.rightMotor.setPower(right * Math.abs(right));
                } else if (reverse == 0) {
                    right = gamepad1.right_stick_y;
                    left = -gamepad1.left_stick_y;

                    robot.leftMotor.setPower(left * Math.abs(left));
                    robot.rightMotor.setPower(right * Math.abs(right));
                }


                if (!robot.bottomLimit.getState()) {
                    if (gamepad2.left_stick_y > 0) {
                        lift = 0;
                    } else {
                        lift = -gamepad2.left_stick_y;
                    }
                } else {
                    lift = -gamepad2.left_stick_y;
                }


                robot.liftMotor.setPower(lift * Math.abs(lift));


                // Use gamepad Y & A raise and lower the arm


                // Use gamepad X & B to open and close the claw
/*
*  if (gamepad1.left_bumper){
                clawPosition += robot.CLAW_SPEED;
            }
            else if (gamepad1.right_bumper) {
                clawPosition -= robot.CLAW_SPEED;
            }
telemetry.addData("claw position is ", clawPosition);

* */
            while (opModeIsActive()) {

                robot.arcadeDrive(-gamepad1.right_stick_y,gamepad1.right_stick_x);

                robot.liftMotor.setPower(gamepad2.left_stick_y);

                if (gamepad2.a){
                    FLIP_POS = TOP;
                    robot.flip.setPosition(FLIP_POS);
                }
                else if (gamepad2.b){
                    FLIP_POS = MID;
                    robot.flip.setPosition(FLIP_POS);
                }
                else if (gamepad2.x){
                    FLIP_POS = BOT;
                    robot.flip.setPosition(FLIP_POS);
                }
                else if (gamepad2.right_bumper){
                    FLIP_POS = FLIPTEMP;
                    FLIP_POS = (FLIPTEMP += Flip_DELTA);

                    robot.flip.setPosition(FLIP_POS);

                }
                else if (gamepad2.left_bumper){
                    FLIP_POS = FLIPTEMP;
                    FLIP_POS = FLIPTEMP -= Flip_DELTA;
                    //FLIP_POS = Range.clip(FLIP_POS, 0.0,1.0);
                    robot.flip.setPosition(FLIP_POS);
                }



                telemetry.addData("claw position is ", clawPosition);


                // Move both servos to new position.
                // robot.armPosition  = Range.clip(robot.armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
                //   robot.arm1.setPosition(robot.armPosition);


                // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
                robot.clawR.setPosition(1.00 - clawPosition);
                robot.clawL.setPosition(clawPosition);
                robot.flipper.setPosition(flipPosition);
                robot.grab.setPosition(hold);
                // Send telemetry message to signify robot running;
                telemetry.addData("arm", "%.2f", robot.armPosition);
//            telemetry.addData("claw",  "%.2f", clawPosition);

                telemetry.update();

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);
            }
        }
    }
