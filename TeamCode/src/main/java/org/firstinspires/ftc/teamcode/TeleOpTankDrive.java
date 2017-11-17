package org.firstinspires.ftc.teamcode;

/*
    Offical Code of AIM Acedemy Robotics Team

    Do not copy any of code unless allowed told to

    This code is for Tank Drive only
    This code also include lift, claw, and flipper

 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TankDrive", group = "9997")
public class TeleOpTankDrive extends LinearOpMode {

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
        double reverse = 1;


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
                reverse = 1;
            } else if (gamepad1.b) {
                reverse = 0;
            }

            if (reverse == 1) {

              //Sets reverse
                right = -gamepad1.right_stick_y;
                left = gamepad1.left_stick_y;

                //Apply to motor
                robot.leftMotor.setPower(left * Math.abs(left));
                robot.rightMotor.setPower(right * Math.abs(right));
            } else if (reverse == 0) {

                //Sets not as reverse
                right = gamepad1.right_stick_y;
                left = -gamepad1.left_stick_y;

                //Applies to motor
                robot.leftMotor.setPower(left * Math.abs(left));
                robot.rightMotor.setPower(right * Math.abs(right));
            }

            //Relic Claw Positioning
            if (gamepad2.left_bumper) {

                hold = 0.0;
            } else if (gamepad2.right_bumper) {

                hold = 1.0;
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
                if (gamepad2.left_stick_y < 0) {
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
