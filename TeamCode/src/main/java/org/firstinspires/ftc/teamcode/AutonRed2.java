
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Team9997Hardware;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name = "AutonRed2", group = "9997")
//@Disabled
public class AutonRed2 extends LinearOpMode {

    /* Declare OpMode members. */
    Team9997Hardware robot = new Team9997Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897);
    static final double DRIVE_SPEED = 0.9;
    static final double SLOW_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;
    static final double LIFT_SPEED = 0.9;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.clawL.setPosition(1.0);    // why is this set to close after initializing?
        robot.clawR.setPosition(0.0);
        sleep(1000);

        robot.arm.setPosition(0.45);
        sleep(1500);

        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
        robot.liftMotor.setPower(0.9);
        sleep(1500);
        robot.liftMotor.setPower(0);


        telemetry.addData("RED", robot.color_sensor.red());
        telemetry.addData("GREEN", robot.color_sensor.green());
        telemetry.addData("BLUE", robot.color_sensor.blue());
        telemetry.addData("ALPHA", robot.color_sensor.alpha());
        telemetry.addData("ARGB", robot.color_sensor.argb());//
        telemetry.update();

        if (robot.color_sensor.red() > robot.color_sensor.blue()) {   // if robot sees red do this code:
            encoderDrive(SLOW_SPEED, 3, -3, 3.0);
            robot.arm.setPosition(0);
            sleep(1000);



            encoderDrive(DRIVE_SPEED, 10, -10, 3.0);

            encoderDrive(DRIVE_SPEED, 9, 9, 3.0);

            encoderDrive(DRIVE_SPEED, 15, -15, 5.0);

            // S1: Forward 47 Inches with 5 Sec timeout
            // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout    Left = 0 riht


            robot.clawL.setPosition(0.5);            // S4: Stop and close the claw.
            robot.clawR.setPosition(0.5);
            sleep(1000);     // pause for servos to move

            encoderDrive(DRIVE_SPEED, -3, 3, 1);

            // encoderDrive(DRIVE_SPEED, 37, -37, 5.0);//move back on to balencing platfrm

            telemetry.addData("Path", "Complete");
            telemetry.update();
            telemetry.addData("color", Integer.toString(robot.color_sensor.alpha()));
            telemetry.update();
        } else {   // if robot sees blue run this code:
            encoderDrive(DRIVE_SPEED, -2, 2, 2.0);
            encoderDrive(DRIVE_SPEED, 2, 2, 2.0);
            encoderDrive(DRIVE_SPEED, -2, -2, 2.0);
            robot.arm.setPosition(0);
            sleep(1000);

            encoderDrive(DRIVE_SPEED, 26, -26, 6.0);

            encoderDrive(DRIVE_SPEED, 10, 10
                    , 4);

            encoderDrive(DRIVE_SPEED, 10, -10, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
            // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout    Left = 0 riht


            robot.clawL.setPosition(0.5);            // S4: Stop and close the claw.
            robot.clawR.setPosition(0.5);
            sleep(1000);     // pause for servos to move

            encoderDrive(DRIVE_SPEED, -3, 3, 1.0);
            // encoderDrive(DRIVE_SPEED, 37, -37, 5.0);//move back on to balencing platfrm

            telemetry.addData("Path", "Complete");
            telemetry.update();
            telemetry.addData("color", Integer.toString(robot.color_sensor.alpha()));
            telemetry.update();
        }


    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    //Experimenting with encoders for lift


    /*public void encoderLift(double speed,
                            double LiftInches,
                            double timeoutS) {

        int newLiftTarget;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int) (LiftInches * COUNTS_PER_INCH);
            robot.liftMotor.setTargetPosition(newLiftTarget);

            // Turn on RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.liftMotor.setPower(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }*/

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always qend the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}