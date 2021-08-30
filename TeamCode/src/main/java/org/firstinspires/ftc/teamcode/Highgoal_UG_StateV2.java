package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "HighGoal_StateV2", group = "autonomous")
public class Highgoal_UG_StateV2 extends LinearOpMode {

    static DcMotor frontleft;
    static DcMotor frontright;
    static DcMotor gatejerry;
    static DcMotor shooterjerry;
    static DcMotor conveyorjerry;
    static DcMotor intakejerry;
    static DcMotor hexjerry;
    static DcMotor armjerry;
    static MoveDirection Direction;
    static AttachmentStates Status;
    static Servo servojerry;
    NormalizedColorSensor colorsensor;
    DistanceSensor distancesensor;
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
    String num_of_rings;
    double speed;
    double checkInterval = 0.04;
    double prevPosition;
    double shootingtime = 3000;

    ElapsedTime ET = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    boolean Box_A;
    boolean Box_B;
    boolean Box_C;

    private static final double SHOOTER_TARGET_SPEED_BASE = 0.65;
    private static final int SHOOTER_ANGULAR_SPEED_RANGE = 60;

    // Box A shooting speed & window
    private static final int SHOOTER_ANGULAR_SPEED_MAX_A = 1540;
    private static final double SHOOTER_TARGET_SPEED_A = 0.65;

    // Box B shooting speed & window
    private static final int SHOOTER_ANGULAR_SPEED_MAX_B = 1540;
    private static final double SHOOTER_TARGET_SPEED_B = 0.65;

    // Box C shooting speed & window
    private static final int SHOOTER_ANGULAR_SPEED_MAX_C = 1540;
    private static final double SHOOTER_TARGET_SPEED_C = 0.65;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AXhILmj/////AAABmZ2G36Eg5k0ThibzOCYNI3414QDSzs2D8IZaPDmv7GTK1DM+1q2KTcH4uAQmWbMIExGi0CtO5JWf2U0nO2HyBuco2BWCXxRi+y3AKuELmddFEb2JYUOIpvTZ7MolJvUdRxhmjIo5Y4N5Vl9uk2tXXZ/5NO7D0vYg/fBgpUVyO/+OnO0UIX3qotxFuCDdN86IlfygQ0p6vLtEnmUIIclVfunY4j3zDlXSbblNTMYPR96a1DjxjrNfldPEHJA+E7u8W0PvdGrtbuEqdwjgbjZjlIT30Vh/sWtaCVaY6WoqNICatk9IyHaw+Cl575F5P6tCXjR4Ib5Cr31YN3RUgLXODgKzyZM5JhRyeNjCPdHqMUqI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double magnification = 1;
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_right_already_pressed = false;

    public void runOpMode() {

        MotorInitialize(
                hardwareMap.dcMotor.get("LeftJerry"),
                hardwareMap.dcMotor.get("RightJerry")
        );

        AttachmentInitialize(
                hardwareMap.dcMotor.get("GateJerry"),
                hardwareMap.dcMotor.get("ShooterJerry"),
                hardwareMap.dcMotor.get("ConveyorJerry"),
                hardwareMap.dcMotor.get("IntakeJerry"),
                hardwareMap.dcMotor.get("HexJerry"),
                hardwareMap.dcMotor.get("ArmJerry"),
                hardwareMap.servo.get("ServoJerry1")
        );

        EncoderInitialize();
        SetDirection(MoveDirection.REVERSE);
        AttachmentSetDirection();

        //AVAILABLE BUT NOT USED - Color sensor (underneath center of robot) and Distance Sensor (right side of robot)
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "ColorJerry1");
        distancesensor = hardwareMap.get(DistanceSensor.class, "DistanceJerry1");
        servojerry = hardwareMap.get(Servo.class, "ServoJerry1");

        //Calibrate gyro
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU = hardwareMap.get(BNO055IMU.class, "imu1");
        IMU.initialize(parameters);

        while (!isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        telemetry.update();

        //Configure IMU
        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

        //Initialize Vuforia and the TensorFlow Object Detect
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        //Read the stack of rings
        ET.reset();

        //Initialize shooter to use close loop speed control
        shooterjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterjerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Grip the wobble goal so it doesn't fall off while the robot is moving
        hexjerry.setPower(-0.6);


        waitForStart();

        while (opModeIsActive()) {

            //Start the shooter now because it takes some time for its speed to stabilize
            AttachmentsControl(AttachmentStates.STARTSHOOTER, SHOOTER_TARGET_SPEED_BASE, 0);
            sleep(500);

            //Move the arm up so it doesn't block the camera
            ArmTurn_RTP(-800);

            //Move forward until we are close to the white line, then turn a bit
            DirectionFollower2(2300, 0.7, 0, 0.0003, 0.00001, 0.05);
            sleep(100);
            servojerry.setPosition(0);
            GyroTurn2(3.5, 0.3, false, true);
            //sleep(200);
            ET.reset();

            //Read the stack of rings
            while (ET.milliseconds() < 2000) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            num_of_rings = recognition.getLabel();
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                    }
                }
                telemetry.addData("# of Rings", num_of_rings);
                telemetry.update();
            }

            //Determine which box to bring the wobble goal to
            if (num_of_rings == "Quad") {
                Box_C = true;
            } else if (num_of_rings == "Single") {
                Box_B = true;
            } else {
                Box_A = true;
            }

            if (Box_A) {

                AttachmentsControl(AttachmentStates.STARTSHOOTER, SHOOTER_TARGET_SPEED_A, 0);

                //move towards box A
                GyroTurn2(70, 0.7, true, false);
                ArmTurn_RTP(-2900);
                DirectionFollower(665, 0.75, 70, 0.0003, 0.00001, 0.05);
                sleep(300);

                //place wobble goal in box A
                ArmClawPosition(1, 200);

                //turn and reverse; lift the arm
                GyroTurn2(40, 0.7, true, true);
                ArmTurn_RTP(-500);

                //reverse until in front of the high goal, then turn to face it
                DirectionFollower(-400, 0.7, 40, 0.0003, 0.00001, 0.05);
                GyroTurn2(-9, 0.7, true, true);

                //shoot 3 rings into high goal
                //Reset shooter speed calculation timer and sample current position (shooter speed should have stabilized by now)
                sleep(2000);
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                //Shoot the 3 rings
                while (ET.milliseconds() < (shootingtime + 1000)) {

                    //If we still haven't shot the rings by now, just shoot anyway
                    if (ET.milliseconds() >= (shootingtime - 200)) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_A, 0);
                    }

                    //Calculate shooter speed once every 0.1 sec
                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    //If the shooter speed is within a range, start launching the rings; else stop the launcher
                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_A - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_A) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_A, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_A, 0);
                        }
                    }
                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                sleep(200);
                AttachmentsControl(AttachmentStates.STOPLAUNCH, 0, 0);
                AttachmentsControl(AttachmentStates.STOPSHOOTER, 0, 0);

                //lower the arm again
                ArmTurn_RTP(-3100);

                //turn and continue reversing
                GyroTurn2(40, 0.7, true, true);
                DirectionFollower(-1950, 0.75, 40, 0.0003, 0.00001, 0.05);

                //turn and move towards 2nd wobble goal
                GyroTurn2(90, 0.7, true, true);
                DirectionFollower(1125, 0.6, 90, 0.0003, 0.00001, 0.05);

                //pick up 2nd wobble goal
                ArmClawPosition(-0.8, 600);
                hexjerry.setPower(-1);
                ArmTurn_RTP(-3000);

                //turn to face box A
                GyroTurn2(22, 0.7, true, true);

                //place second wobble goal in box a and park on line
                DirectionFollower(1750, 0.75, 22, 0.0003, 0.00001, 0.05);
                ArmClawPosition(0.7, 400);
                ArmTurn_RTP(-1800);

            } else if (Box_B) {

                AttachmentsControl(AttachmentStates.STARTSHOOTER, SHOOTER_TARGET_SPEED_B, 0);

                //move towards box B
                servojerry.setPosition(1);
                ArmTurn_RTP(-2900);
                DirectionFollower2(780, 0.75, 0, 0.0003, 0.00001, 0.05);
                sleep(200);
                GyroTurn2(25, 0.7, true, true);
                sleep(200);

                //place wobble goal in box B
                ArmClawPosition(1, 200);

                //get ready to reverse; raise arm up
                //AttachmentsControl(AttachmentStates.STARTINTAKE, SHOOTER_TARGET_SPEED_B, 0);
                GyroTurn2(-23, 0.7, true, true);
                ArmTurn_RTP(-500);

                //reverse until in front of the high goal, then turn to face it
                servojerry.setPosition(0);
                DirectionFollower2(-900, 0.5, -23, 0.0003, 0.00001, 0.05);
                GyroTurn2(7, 0.7, true, true);

                //shoot 3 rings into high goal
                //Reset shooter speed calculation timer and sample current position (shooter speed should have stabilized by now)
                sleep(2000);
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                //Shoot the three rings
                while (ET.milliseconds() < (shootingtime + 1000)) {

                    //If we still haven't shot the rings by now, just shoot anyway
                    if (ET.milliseconds() >= (shootingtime - 200)) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_B, 0);
                    }

                    //Calculate shooter speed once every 0.1 sec
                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    //If the shooter speed is within a range, start launching the rings; else stop the launcher
                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_B - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_B) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_B, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_B, 0);
                        }
                    }
                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                //close the gate and lower the arm
                sleep(200);
                AttachmentsControl(AttachmentStates.STARTINTAKE, SHOOTER_TARGET_SPEED_B, 0);
                ArmTurn_RTP(-2700);
                servojerry.setPosition(1);

                //turn CCW so the intake faces the ring on the floor; continue reversing to pick up the ring
                GyroTurn2(-23, 0.7, false, true);
                DirectionFollower2(-1260, 0.6, -23, 0.0003, 0.00001, 0.05);

                //Turn and start lowering the arm to pick up the 2nd wobble goal
                GyroTurn2(90, 0.7, false, true);
                ArmTurn_RTP(-3100);
                sleep(300);
                GyroTurn2(151, 0.7, false, true);
                ArmClawPosition(-0.8, 400);
                hexjerry.setPower(-1);
                ArmTurn_RTP(-1100);
                //AttachmentsControl(AttachmentStates.STOPINTAKE, SHOOTER_TARGET_SPEED_B, 0);

                //Turn back and get into position to shoot the ring
                GyroTurn2(-26, 0.7, true, true);
                DirectionFollower2(640, 0.6, -26, 0.0003, 0.00001, 0.05);
                GyroTurn2(-6, 0.7, true, true);
                DirectionFollower2(160, 0.6, -6, 0.0003, 0.00001, 0.05);
                sleep(200);

                //Reset shooter speed calculation timer and sample current position
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                //shoot the ring or rings (if leftover from first attempt)
                while (ET.milliseconds() < 2000) {

                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_B - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_B) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_B, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_B, 0);
                        }
                    }

                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                sleep(200);
                AttachmentsControl(AttachmentStates.STOPLAUNCH, 0, 0);
                AttachmentsControl(AttachmentStates.STOPSHOOTER, 0, 0);

                //park on white line and lower 2nd wobble goal into Box B
                ArmTurn_RTP(-3100);
                DirectionFollower2(730, 0.6, 0, 0.0003, 0.00001, 0.05);
                sleep(200);
                ArmClawPosition(1, 200);
                ArmTurn_RTP(-100);
                sleep(500);

            } else if (Box_C) {

                AttachmentsControl(AttachmentStates.STARTSHOOTER, SHOOTER_TARGET_SPEED_C, 0);

                //move towards box C
                ArmTurn_RTP(-2900);
                DirectionFollower2(1230, 0.75, 0, 0.0003, 0.00001, 0.05);
                GyroTurn2(43, 0.8, false, true);
                DirectionFollower2(1100, 0.75, 45, 0.0003, 0.00001, 0.05);

                //place wobble goal in box C
                ArmClawPosition(1, 200);

                //reverse, lift the arm and turn
                DirectionFollower2(-360, 0.7, 45, 0.0003, 0.00001, 0.05);
                ArmTurn_RTP(-500);
                GyroTurn2(-6, 0.8, true, true);

                //open the gate to get ready for shooting and reverse
                servojerry.setPosition(0);
                DirectionFollower2(-1550, 0.75, -7, 0.0003, 0.00001, 0.05);
                sleep(100);

                //shoot 3 rings into the high goal
                //Reset shooter speed calculation timer and sample current position (shooter speed should have stabilized by now)
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                while (ET.milliseconds() < shootingtime) {

                    //If we still haven't shot the rings by now, just shoot anyway
                    if (ET.milliseconds() >= (shootingtime - 200)) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    }

                    //Calculate shooter speed once every 0.1 sec
                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    //If the shooter speed is within a range, start launching the rings; else stop the launcher
                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_C - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_C) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                        }
                    }
                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                //prepare the arm to pick up the second wobble goal and close the gate
                intakejerry.setPower(0);
                gatejerry.setPower(0.8);
                conveyorjerry.setPower(0.8);
                ArmTurn_RTP(-2700);
                servojerry.setPosition(1);

                //reverse fast when hitting the stack of rings so they scatter on the floor
                DirectionFollower2(-520, 0.95, -6, 0.0003, 0.00001, 0.05);
                sleep(200);

                //start the intake and backup to pick up the rings
                AttachmentsControl(AttachmentStates.STARTINTAKE, SHOOTER_TARGET_SPEED_C, 0);
                DirectionFollower2(-650, 0.65, -6, 0.0003, 0.00001, 0.05);

                //Turn towards 2nd wobble goal
                ArmTurn_RTP(-3150);
                GyroTurn2(141, 0.8, false, true);
                sleep(200);

                //stop the intake to prevent accidentally picking up too many rings when we turn back to face the high goal
                intakejerry.setPower(0);

                //grab the second wobble goal and lift it up
                ArmClawPosition(-0.8, 200);
                hexjerry.setPower(-1);
                ArmTurn_RTP(-1000);

                // Turn back to face the high goal
                GyroTurn2(-6, 0.8, true, true);

                //move forward to shoot the rings into the high goal
                DirectionFollower2(1000, 0.75, -6, 0.0003, 0.00001, 0.05);

                conveyorjerry.setPower(-0.9);
                gatejerry.setPower(-0.9);
                //sleep(300);

                //open the gate
                servojerry.setPosition(0);
                sleep(100);

                //Reset shooter speed calculation timer and sample current position
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                //shoot the rings
                while (ET.milliseconds() < 1750) {

                    //if (ET.milliseconds() >= 1750) {
                    //AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    //}

                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_C - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_C) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                        }
                    }
                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                //close the gate and start the intake
                AttachmentsControl(AttachmentStates.STARTINTAKE, SHOOTER_TARGET_SPEED_C, 0);

                //Reverse to pick up more rings off the floor
                DirectionFollower2(-1650, 0.75, -5, 0.0003, 0.00001, 0.05);
                servojerry.setPosition(1);
                DirectionFollower2(1650, 0.75, -6, 0.0003, 0.00001, 0.05);

                //open the gate again
                servojerry.setPosition(0);
                sleep(100);

                //Reset shooter speed calculation timer and sample current position
                timer.reset();
                prevPosition = shooterjerry.getCurrentPosition();
                ET.reset();

                //shoot the rings
                while (ET.milliseconds() < 2000) {

                    if (ET.milliseconds() >= 1750) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    }

                    if (timer.time() > checkInterval) {
                        speed = (double) (shooterjerry.getCurrentPosition() - prevPosition) / timer.time();
                        prevPosition = shooterjerry.getCurrentPosition();
                        timer.reset();
                    }

                    if (speed >= (SHOOTER_ANGULAR_SPEED_MAX_C - SHOOTER_ANGULAR_SPEED_RANGE) && speed <= SHOOTER_ANGULAR_SPEED_MAX_C) {
                        AttachmentsControl(AttachmentStates.LAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                    } else {
                        if (conveyorjerry.getPower() != 0) {
                            AttachmentsControl(AttachmentStates.STOPLAUNCH, SHOOTER_TARGET_SPEED_C, 0);
                        }
                    }
                    telemetry.addData("Shooter Speed", speed);
                    telemetry.update();
                }

                //AttachmentsControl(AttachmentStates.STOPLAUNCH, 0, 0);
                //AttachmentsControl(AttachmentStates.STOPSHOOTER, 0, 0);

                //move forward and drop 2nd wobble goal in Box C
                ArmTurn_RTP(-2700);
                GyroTurn2(14, 0.75, false, true);
                DirectionFollower2(1550, 0.75, 14, 0.0003, 0.00001, 0.05);
                ArmClawPosition(1, 200);
                //sleep(100);

                //park on the white line
                DirectionFollower2(-450, 0.75, 14, 0.0003, 0.00001, 0.05);
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            break;
        }
    }



    private void MotorInitialize(DcMotor LeftJerry,
                                 DcMotor RightJerry

    ) {

        frontleft = LeftJerry;
        frontright = RightJerry;
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private static void AttachmentInitialize (DcMotor GateJerry,
                                              DcMotor ShooterJerry,
                                              DcMotor ConveyorJerry,
                                              DcMotor IntakeJerry,
                                              DcMotor HexJerry,
                                              DcMotor ArmJerry,
                                              Servo ServoJerry1) {

        intakejerry = IntakeJerry;
        gatejerry = GateJerry;
        shooterjerry = ShooterJerry;
        conveyorjerry = ConveyorJerry;
        hexjerry = HexJerry;
        armjerry = ArmJerry;
        servojerry = ServoJerry1;

        armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servojerry.setPosition(0); // gate is physically opened at the beginning (i.e. position 0)

    }

    private void SetMotorPower(double x) {

        frontleft.setPower(x);
        frontright.setPower(x);

    }

    private void MotorTurn(double x, double y) {

        frontleft.setPower(x);
        frontright.setPower(y);

    }

    // For use with RUN_WITHOUT_ENCODER mode
    private void ArmTurn(double power, long time) {

        armjerry.setPower(power);
        sleep(time);

        // For keeping the arm in place. Zero power will cause the arm to drop due to gravity
        // Only works if the arm is not carrying something
        if (power > 0) {
            armjerry.setPower(-0.04);
        }

    }

    // For use with RUN_TO_POSITION mode
    private void ArmTurn_RTP(int arm_pos_chg) {

        //armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armjerry.setTargetPosition(arm_pos_chg);
        armjerry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armjerry.setPower(0.7);

    }

    private void ArmClawPosition(double power, long time) {

        hexjerry.setPower(power);
        sleep(time);
        hexjerry.setPower(0);

    }

    private static void SetDirection (MoveDirection direction) {

        Direction = direction;

        if (Direction == MoveDirection.FORWARD) {

            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleft.setDirection(DcMotorSimple.Direction.FORWARD);

        } else {

            frontright.setDirection(DcMotorSimple.Direction.FORWARD);
            frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }

    private void AttachmentSetDirection () {

        shooterjerry.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        gatejerry.setDirection(DcMotorSimple.Direction.FORWARD);
        intakejerry.setDirection(DcMotorSimple.Direction.REVERSE);
        hexjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        //armjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        servojerry.setDirection(Servo.Direction.FORWARD);

    }

    private void AttachmentsControl (AttachmentStates status, double powerofshooter, long amount) {

        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            shooterjerry.setPower(powerofshooter);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            shooterjerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            servojerry.setPosition(1); // Close the gate
            //gatejerry.setPower(0.3);
            gatejerry.setPower(0.8);
            intakejerry.setPower(1);
            conveyorjerry.setPower(0.8);

        } else if (Status == AttachmentStates.STOPINTAKE) {

            servojerry.setPosition(1); // Close the gate
            intakejerry.setPower(0);
            conveyorjerry.setPower(0);
            gatejerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            servojerry.setPosition(0);
            intakejerry.setPower(0.5);
            gatejerry.setPower(0.9);
            conveyorjerry.setPower(0.9);
            sleep(amount);

        }

        if (Status == AttachmentStates.LAUNCHPREP) {

            servojerry.setPosition(1);
            gatejerry.setPower(-0.2);
            conveyorjerry.setPower(0.2);
            intakejerry.setPower(0.4);

        }

        if (Status == AttachmentStates.STOPLAUNCH) {

            intakejerry.setPower(0);
            conveyorjerry.setPower(0);
            gatejerry.setPower(0);

        }
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

    private void GyroTurn (double angledegree, double power) {

        SetDirection(MoveDirection.REVERSE);

        if (angledegree > GyroContinuity()) {

            while (GyroContinuity() <= angledegree) {

                MotorTurn(power, -power);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (power > 0.5) {
                while (GyroContinuity() >= angledegree) {

                    MotorTurn(-0.5, 0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }

        } else {

            while (GyroContinuity() >= angledegree) {

                MotorTurn(-power, power);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (power > 0.5) {
                while (GyroContinuity() <= angledegree) {

                    MotorTurn(0.5, -0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }
        }
        SetMotorPower(0);
        sleep(200);
    }

    private void GyroTurn2 (double angledegree, double power, boolean dualstageturning, boolean turninplace) {

        SetDirection(MoveDirection.REVERSE);

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

    private void MoveEncoder(double TargetPosition, double Power) {

        if (TargetPosition > 0) {

            //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SetDirection(MoveDirection.REVERSE);
            SetMotorPower(Power);

            while (frontright.getCurrentPosition() < TargetPosition) {

                telemetry.addData("Left Motor", frontright.getCurrentPosition());
                telemetry.update();

            }

            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        else {

            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SetDirection(MoveDirection.REVERSE);
            SetMotorPower(Power);

            while (frontright.getCurrentPosition() < -TargetPosition) {

                telemetry.addData("Left Motor", frontright.getCurrentPosition());
                telemetry.update();

            }

            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        SetMotorPower(0);
        sleep(200);

    }

    private static void EncoderInitialize() {

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void DirectionFollower(double targetdistance, double power, double TargetDirection,
                                   double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.REVERSE);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", frontright.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-frontright.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", -frontright.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);
        GyroTurn(TargetDirection, 0.3);

    }

    private void DirectionFollower2(double targetdistance, double power, double TargetDirection,
                                   double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.REVERSE);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", frontright.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-frontright.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", -frontright.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        //sleep(100);
        //GyroTurn(TargetDirection, 0.3);

    }

    private void Wallfollower (double targetdistance, double power, double kp_in,
                               double ki_in, double kd_in, double CMfromWall, double targetdirection) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.REVERSE);

            SteeringOutput = PID.PID_Control(CMfromWall, kp_in, ki_in, kd_in, distancesensor.getDistance(DistanceUnit.CM));
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Distance From Wall", CMfromWall);
            telemetry.addData("Encoder", frontright.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.update();

        }

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);
        GyroTurn(targetdirection, 0.6);

    }

    private int WhiteDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(70);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int White = 1;
        int Unkwown = 0;

        if (HSV[1] <= 0.25) {
            if (HSV[2] >= 0.93) {
                telemetry.addData("Color:", "White");
                telemetry.update();
                return White;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            return Unkwown;
        }
    }

    private void WhiteDirectionFollower(double power, double TargetDirection,
                                   double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (WhiteDetector() != 1) {

            SetDirection(MoveDirection.REVERSE);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        SetMotorPower(0);
        sleep(100);
        GyroTurn(TargetDirection, 0.3);

    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }
}