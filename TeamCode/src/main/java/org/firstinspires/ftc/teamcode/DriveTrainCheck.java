package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@TeleOp(name = "DriveTrainCheckTeleOp", group = "Linear Opmode")
public class DriveTrainCheck extends LinearOpMode {

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
    double shootingtime = 3800;

    boolean Box_A;
    boolean Box_B;
    boolean Box_C;

    private static final int SHOOTER_ANGULAR_SPEED_MAX = 1650;
    private static final int SHOOTER_ANGULAR_SPEED_RANGE = 60;
    private static final double SHOOTER_TARGET_SPEED = 0.68;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AXhILmj/////AAABmZ2G36Eg5k0ThibzOCYNI3414QDSzs2D8IZaPDmv7GTK1DM+1q2KTcH4uAQmWbMIExGi0CtO5JWf2U0nO2HyBuco2BWCXxRi+y3AKuELmddFEb2JYUOIpvTZ7MolJvUdRxhmjIo5Y4N5Vl9uk2tXXZ/5NO7D0vYg/fBgpUVyO/+OnO0UIX3qotxFuCDdN86IlfygQ0p6vLtEnmUIIclVfunY4j3zDlXSbblNTMYPR96a1DjxjrNfldPEHJA+E7u8W0PvdGrtbuEqdwjgbjZjlIT30Vh/sWtaCVaY6WoqNICatk9IyHaw+Cl575F5P6tCXjR4Ib5Cr31YN3RUgLXODgKzyZM5JhRyeNjCPdHqMUqI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
                hardwareMap.dcMotor.get("ArmJerry")

        );

        EncoderInitialize();
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SetDirection(MoveDirection.FORWARD);
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

        //Grip the wobble goal so it doesn't fall off while the robot is moving
        hexjerry.setPower(-0.6);

        //Create elapsed timers for use during shooting
        ElapsedTime ET = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                MoveEncoder(3500, 0.5);
            }

            if (gamepad1.dpad_down) {
                MoveEncoder(-3500, 0.5);
            }

            if (gamepad1.dpad_left) {
                SetDirection(MoveDirection.REVERSE);
                frontright.setPower(0.5);
                sleep(2000);
                frontright.setPower(0);
            }

            if (gamepad1.dpad_right) {
                SetDirection(MoveDirection.REVERSE);
                frontleft.setPower(0.5);
                sleep(2000);
                frontleft.setPower(0);
            }

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
                                              DcMotor ArmJerry) {

        intakejerry = IntakeJerry;
        gatejerry = GateJerry;
        shooterjerry = ShooterJerry;
        conveyorjerry = ConveyorJerry;
        hexjerry = HexJerry;
        armjerry = ArmJerry;

        armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        gatejerry.setDirection(DcMotorSimple.Direction.REVERSE);
        intakejerry.setDirection(DcMotorSimple.Direction.FORWARD);
        hexjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        //armjerry.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void AttachmentsControl (AttachmentStates status, double powerofshooter, long amount) {

        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            shooterjerry.setPower(powerofshooter);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            shooterjerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            gatejerry.setPower(-0.25); //"Close" the gate
            intakejerry.setPower(1); // Prev: 0.9
            conveyorjerry.setPower(0.45); // Prev: 0.8

        } else if (Status == AttachmentStates.STOPINTAKE) {

            intakejerry.setPower(0);
            conveyorjerry.setPower(0);
            gatejerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            intakejerry.setPower(1);
            gatejerry.setPower(0.45);
            conveyorjerry.setPower(0.45);
            sleep(amount);

        }

        if (Status == AttachmentStates.LAUNCHPREP) {

            gatejerry.setPower(-0.25); //"Close" the gate
            conveyorjerry.setPower(0.1);
            intakejerry.setPower(0.4);

        }

        if (Status == AttachmentStates.STOPLAUNCH) {

            intakejerry.setPower(0);
            gatejerry.setPower(0);
            conveyorjerry.setPower(0);

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

        SetDirection(MoveDirection.FORWARD);

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

    private void MoveEncoder(double TargetPosition, double Power) {

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (TargetPosition > 0) {

            SetDirection(MoveDirection.REVERSE);
            //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SetMotorPower(Power);

            while (frontright.getCurrentPosition() < TargetPosition) {

                telemetry.addData("Left Motor", frontright.getCurrentPosition());
                telemetry.update();

            }

            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        else {

            SetDirection(MoveDirection.FORWARD);
            //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SetMotorPower(Power);

            while (-frontright.getCurrentPosition() > TargetPosition) {

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
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

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

            SetDirection(MoveDirection.REVERSE);

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
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

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

            SetDirection(MoveDirection.REVERSE);

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
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (frontright.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

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
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (WhiteDetector() != 1) {

            SetDirection(MoveDirection.FORWARD);

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