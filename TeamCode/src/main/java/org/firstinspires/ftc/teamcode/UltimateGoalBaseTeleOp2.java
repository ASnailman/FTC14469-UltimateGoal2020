package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *if adam is rading this, do ur homework
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.*
 *newest versieoin
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Number 6: man urinates on fellow passenger for not being able to smoke on the train
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * EXTRA CODE
 *  (Servo.class, "Gripper")
 */

@TeleOp(name="Teleop2", group="Linear Opmode")
//@Disabled
public class UltimateGoalBaseTeleOp2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static DcMotor LeftJerry;
    static DcMotor RightJerry;
    private DcMotor ShooterJerry;
    private DcMotor ConveyorJerry;
    private DcMotor IntakeJerry;
    private DcMotor GateJerry;
    private DcMotor HexJerry;
    private DcMotor ArmJerry;
    static MoveDirection Direction;
    static AttachmentStates Status;
    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;
    double Lpower;
    double Rpower;
    double SteeringOutput;
    double speed;
    double prevPosition;
    double checkInterval = 0.04;
    boolean button_dpad_down_already_pressed = false;
    boolean button_a_already_pressed = false;
    boolean lower_arm = true;
    boolean button_x_already_pressed = false;
    boolean clamp_open = true;

    double gatePower;
    double gate;
    double gateneg;

    double conveyorPower;
    double conveyor;
    double conveyorneg;

    double armpower;
    double armpos;
    double armneg;

    double shooter_target_speed = HG_SHOOTER_TARGET_SPEED;
    double shooter_angular_speed_max = HG_SHOOTER_ANGULAR_SPEED_MAX;

    double leftPower;
    double leftPowerB;
    double rightPower;
    double rightPowerB;

    double drive;
    double turnplace;


    private static final int SHOOTER_ANGULAR_SPEED_RANGE = 60;
    private static final int HG_SHOOTER_ANGULAR_SPEED_MAX = 1670;
    private static final double HG_SHOOTER_TARGET_SPEED = 0.68;
    private static final int PS_SHOOTER_ANGULAR_SPEED_MAX = 1460;
    private static final double PS_SHOOTER_TARGET_SPEED = 0.6;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftJerry = hardwareMap.get(DcMotor.class, "LeftJerry");
        RightJerry = hardwareMap.get(DcMotor.class, "RightJerry");
        ShooterJerry = hardwareMap.get(DcMotor.class, "ShooterJerry");
        ConveyorJerry = hardwareMap.get(DcMotor.class, "ConveyorJerry");
        IntakeJerry = hardwareMap.get(DcMotor.class, "IntakeJerry");
        GateJerry = hardwareMap.get(DcMotor.class, "GateJerry");
        HexJerry = hardwareMap.get(DcMotor.class, "HexJerry");
        ArmJerry = hardwareMap.get(DcMotor.class, "ArmJerry");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftJerry.setDirection(DcMotor.Direction.FORWARD);
        RightJerry.setDirection(DcMotor.Direction.REVERSE);
        ShooterJerry.setDirection(DcMotor.Direction.FORWARD);
        ShooterJerry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ConveyorJerry.setDirection(DcMotor.Direction.FORWARD);
        IntakeJerry.setDirection(DcMotor.Direction.FORWARD);
        GateJerry.setDirection(DcMotor.Direction.FORWARD);
        HexJerry.setDirection(DcMotor.Direction.FORWARD);
        ArmJerry.setDirection(DcMotor.Direction.FORWARD);
        ArmJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU = hardwareMap.get(BNO055IMU.class, "imu1");
        //IMU.initialize(parameters);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        telemetry.update();

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;


        //Initialize shooter to use close loop speed control
        ShooterJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Start the shooter now because it takes some time for its speed to stabilize
        AttachmentsControl(AttachmentStates.STARTSHOOTER, shooter_target_speed, 0);

        prevPosition = ShooterJerry.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // Calculate flywheel speed
                if (timer.time() > checkInterval) {
                    speed = (double) (ShooterJerry.getCurrentPosition() - prevPosition) / timer.time();
                    prevPosition = ShooterJerry.getCurrentPosition();
                    timer.reset();
                }

                // Shooter speed selection (Off, High Goal or PowerShot)
                if (gamepad2.dpad_down) {
                    shooter_target_speed = 0;
                } else if (gamepad2.dpad_left) {
                    shooter_target_speed = PS_SHOOTER_TARGET_SPEED;
                    ShooterJerry.setPower(shooter_target_speed);
                    shooter_angular_speed_max = PS_SHOOTER_ANGULAR_SPEED_MAX;
                } else if (gamepad2.dpad_right) {
                    shooter_target_speed = HG_SHOOTER_TARGET_SPEED;
                    ShooterJerry.setPower(shooter_target_speed);
                    shooter_angular_speed_max = HG_SHOOTER_ANGULAR_SPEED_MAX;
                }

                // Speed-based auto shooting
                if (gamepad2.left_bumper)  {

                    //shoot the three rings
                    while (gamepad2.left_bumper) {

                        if (timer.time() > checkInterval) {
                            speed = (double) (ShooterJerry.getCurrentPosition() - prevPosition) / timer.time();
                            prevPosition = ShooterJerry.getCurrentPosition();
                            timer.reset();
                        }

                        if (speed >= (shooter_angular_speed_max - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= shooter_angular_speed_max) {
                            AttachmentsControl(AttachmentStates.LAUNCH, shooter_target_speed, 0);
                        } else {
                            if (ConveyorJerry.getPower() != 0) {
                                AttachmentsControl(AttachmentStates.STOPLAUNCH, shooter_target_speed, 0);
                            }
                        }

                        telemetry.addData("Shooter Speed", speed);
                        telemetry.update();
                    }
                }
                else if (gatePower == 0 && conveyorPower == 0) {
                     AttachmentsControl(AttachmentStates.STOPLAUNCH, shooter_target_speed, 0);
                }


                // One button press auto arm positioning
                if (button_dpad_down_already_pressed == false) {
                    if (gamepad2.b) {
                        if (lower_arm == true) {
                            ArmJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            sleep(1000);
                            ArmClawPosition(-1, 500);
                            ArmTurn_RTP(-3500);
                            lower_arm = false;
                        }
                        else {
                            ArmClawPosition(1, 500);
                            ArmTurn_RTP(-1500);
                            lower_arm = true ;
                        }
                        button_dpad_down_already_pressed = true;
                    }
                }
                else {
                    if (!gamepad2.b) {

                        button_dpad_down_already_pressed = false;
                    }
                }


                // One button press clamp control
                if (button_x_already_pressed == false) {
                    if (gamepad2.x) {
                        if (clamp_open == true) {
                            HexJerry.setPower(-1);
                            clamp_open = false;
                        } else {
                            HexJerry.setPower(1);
                            clamp_open = true;
                        }
                        button_x_already_pressed = true;
                    }
                }
                else {
                    if (!gamepad2.x) {
                        button_x_already_pressed = false;
                    }
                }


                // Manual arm control
                armpos = gamepad2.right_trigger;
                armneg = gamepad2.left_trigger;
                armpower =  Range.clip(armpos  - armneg, -1.0, 1.0);

                if (armpower != 0) {
                    ArmJerry.setPower(armpower);
                    ArmJerry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                // Manual gate control
                gate = gamepad2.right_stick_y;
                gateneg = -gamepad2.right_stick_y;
                gatePower = Range.clip(gate - gateneg,-1, 1);
                GateJerry.setPower(gatePower);

                // Manual conveyor control
                conveyor = gamepad2.left_stick_y;
                conveyorneg = -gamepad2.left_stick_y;
                conveyorPower = Range.clip(conveyor - conveyorneg, -1, 1);
                ConveyorJerry.setPower(conveyorPower);

                // Intake control (Off, Forward, Reverse)
                if (gamepad2.a) {
                    IntakeJerry.setPower(0);
                } else if (gamepad2.y) {
                    IntakeJerry.setPower(1);
                } else if (gamepad2.right_bumper) {
                    IntakeJerry.setPower(-1);
                }


                // Drive train control
                drive = -gamepad1.left_stick_y;
                turnplace = gamepad1.right_stick_x;

                leftPower = Range.clip(drive  + turnplace,  -1.0, 1.0);
                leftPowerB = Range.clip(drive +turnplace, -0.1,0.1 );
                rightPower = Range.clip(drive/1.07  - turnplace, -1.0, 1.0);
                rightPowerB = Range.clip(drive/1.07 - turnplace, -0.1, 0.1);

                if (gamepad1.right_bumper) {
                    LeftJerry.setPower(leftPowerB);
                    RightJerry.setPower(rightPowerB);
                }
                else {
                    LeftJerry.setPower(leftPower);
                    RightJerry.setPower(rightPower);
                }


                // One button press auto positioning for each PowerShot target
                if (gamepad1.x) {
                    GyroTurn2(-19, 0.5, true, true);
                }
                if (gamepad1.y) {
                    GyroTurn2(-12, 0.5, true, true);
                }
                if (gamepad1.b) {
                    GyroTurn2(-8, 0.5, true, true);
                }

                if (button_a_already_pressed == false) {
                    if (gamepad1.a) {

                        IMU.initialize(parameters);
                        sleep(500);
                        //while (!isStopRequested() && !IMU.isGyroCalibrated()) {
                            //sleep(50);
                            //idle();
                        //}
                        button_a_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.a) {
                        button_a_already_pressed = false;
                    }
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Power", shooter_target_speed);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.addData("Amount", ArmJerry.getCurrentPosition());
                telemetry.addData("Shooter Speed", speed);
                telemetry.update();
            }
        }
    }
    private void ArmClawPosition(double power, long time) {

        HexJerry.setPower(power);
        sleep(time);
        HexJerry.setPower(0);

    }

    private void AttachmentsControl (AttachmentStates status, double powerofshooter, long amount) {

        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            ShooterJerry.setPower(powerofshooter);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            ShooterJerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            GateJerry.setPower(0.5); //"Close" the gate
            IntakeJerry.setPower(0.9); // Prev: 1
            ConveyorJerry.setPower(-0.9); // Prev: 0.8

        } else if (Status == AttachmentStates.STOPINTAKE) {

            IntakeJerry.setPower(0);
            ConveyorJerry.setPower(0);
            GateJerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            IntakeJerry.setPower(1);
            GateJerry.setPower(-0.9);
            ConveyorJerry.setPower(-0.9);
            sleep(amount);

        }

        if (Status == AttachmentStates.LAUNCHPREP) {

            GateJerry.setPower(0.5); //"Close" the gate
            ConveyorJerry.setPower(-0.2);
            IntakeJerry.setPower(0.4);

        }

        if (Status == AttachmentStates.STOPLAUNCH) {

            IntakeJerry.setPower(0);
            GateJerry.setPower(0);
            ConveyorJerry.setPower(0);

        }
    }

    private void ArmTurn_RTP(int arm_pos_chg) {

        //armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJerry.setTargetPosition(arm_pos_chg);
        ArmJerry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmJerry.setPower(0.7);

    }


    private void MotorTurn(double x, double y) {

        LeftJerry.setPower(x);
        RightJerry.setPower(y);

    }
    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;

    }
    private void SetMotorPower(double x) {

        LeftJerry.setPower(x);
        RightJerry.setPower(x);

    }

    private void GyroTurn2 (double angledegree, double power, boolean dualstageturning, boolean turninplace) {

        SetDirection(MoveDirection.FORWARD);

        if (angledegree > GyroContinuity()) {

            while (GyroContinuity() <= angledegree) {

                if (turninplace==true) {

                    MotorTurn(power, -power);
                }
                else {
                    MotorTurn(power, 0.02);
                }
                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (dualstageturning==true) {
                while (GyroContinuity() >= angledegree) {

                    MotorTurn(-0.5, 0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }

        } else {

            while (GyroContinuity() >= angledegree) {

                if (turninplace==true) {

                    MotorTurn(-power, power);
                }
                else {
                    MotorTurn(0.02, power);
                }

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (dualstageturning==true) {
                while (GyroContinuity() <= angledegree) {

                    MotorTurn(0.5, -0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }
        }
        SetMotorPower(0);
        // sleep(200);
    }

    private void DirectionFollower2(double targetdistance, double power, double TargetDirection,
                                    double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        RightJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (RightJerry.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            LeftJerry.setPower(Lpower);
            RightJerry.setPower(Rpower);

            telemetry.addData("Encoder", RightJerry.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-RightJerry.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.REVERSE);

            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            LeftJerry.setPower(Lpower);
            RightJerry.setPower(Rpower);

            telemetry.addData("Encoder", -RightJerry.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        RightJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        //sleep(100);
        //GyroTurn(TargetDirection, 0.3);

    }
    private static void SetDirection (MoveDirection direction) {

        Direction = direction;

        if (Direction == MoveDirection.FORWARD) {

            RightJerry.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftJerry.setDirection(DcMotorSimple.Direction.FORWARD);

        } else {

            RightJerry.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftJerry.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }



}



