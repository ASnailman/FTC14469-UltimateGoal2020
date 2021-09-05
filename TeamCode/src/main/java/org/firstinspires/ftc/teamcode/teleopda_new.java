package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * EXTRA CODE
 *  (Servo.class, "Gripper")
 */

@TeleOp(name="MecanumDrive", group="MecanumDrive")
//@Disabled
public class teleopda_new extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor BackLeft;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor FrontRight;
    //DcMotor LeftRail = null;
    //DcMotor RightRail = null;
    //DcMotor AngleMotor = null;
    //Servo Gripper = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        //LeftRail = hardwareMap.get(DcMotor.class, "LeftRail");
        //RightRail = hardwareMap.get(DcMotor.class, "RightRail");
        //AngleMotor = hardwareMap.get(DcMotor.class, "AngleMotor");
        //Gripper = hardwareMap.get(Servo.class, "Gripper");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        //RightRail.setDirection(DcMotor.Direction.REVERSE);
        //AngleMotor.setDirection(DcMotor.Direction.FORWARD);
        //LeftRail.setDirection(DcMotor.Direction.FORWARD);
        //AngleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Gripper.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        if (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                //if (gamepad2.y) {
                    //Gripper.setPosition(0.8);
                //} else if (gamepad2.b) {
                    //Gripper.setPosition(0);
                //}

                // Setup a variable for each drive wheel to save power level for telemetry
                double leftBackPower;
                double rightBackPower;
                double leftFrontPower;
                double rightFrontPower;

                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.


                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = -gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;
                leftBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
                rightBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
                leftFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
                rightFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
                //RightRail.setPower(-(gamepad2.left_stick_y)*2);
                //LeftRail.setPower((gamepad2.left_stick_y)*2);
                //AngleMotor.setPower((gamepad2.right_stick_y));



                //            // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

                // Send calculated power to wheels
                BackLeft.setPower(leftBackPower);
                BackRight.setPower(rightBackPower);
                FrontLeft.setPower(leftFrontPower);
                FrontRight.setPower(rightFrontPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //   telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
    }

}
