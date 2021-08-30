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

@Autonomous(name = "Powershot", group = "autonomous")
public class Powershot_UG extends LinearOpMode {

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

    boolean Box_A;
    boolean Box_B;
    boolean Box_C;

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
        SetDirection(MoveDirection.FORWARD);
        AttachmentSetDirection();
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "ColorJerry1");
        distancesensor = hardwareMap.get(DistanceSensor.class, "DistanceJerry1");

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

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        hexjerry.setPower(-0.6);

        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            AttachmentsControl(AttachmentStates.STARTSHOOTER, 0.4);
            DirectionFollower(500, 0.9, 0, 0.0003, 0.000001, 0.03);
            GyroTurn(-10, 0.3);
            DirectionFollower(2700, 0.9, -10, 0.0003, 0.000001, 0.03);
            SetMotorPower(0);
            sleep(400);
            GyroTurn(-11, 0.3);
            AttachmentsControl(AttachmentStates.LAUNCH, 0.62);
            GyroTurn(-8, 0.3);
            AttachmentsControl(AttachmentStates.LAUNCH, 0.62);
            GyroTurn(-5, 0.3);
            AttachmentsControl(AttachmentStates.LAUNCH, 0.62);
            AttachmentsControl(AttachmentStates.STOPSHOOTER, 0);
            GyroTurn(0, 0.3);
            ET.reset();

            while (ET.milliseconds() < 1500) {
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
                        telemetry.update();
                    }
                }
            }

            if (num_of_rings == "Quad") {
                Box_C = true;
            } else if (num_of_rings == "Single") {
                Box_B = true;
            } else {
                Box_A = true;
            }
            if (Box_A) {

                //face box a
                GyroTurn(40, 1);
                DirectionFollower(3100, 1, 40, 0.0003, 0.000001, 0.03);
                GyroTurn(50, 0.5);
                //place wobble goal in box a
                ArmTurn(0.8, 720);
                sleep(200);
                ArmClawPosition(0.5, 600);
                ArmTurn(-0.65, 470);
                //back up; turn towards right red line
                GyroTurn(40, 0.5);
                DirectionFollower(-5485, 1, 40, 0.0003, 0.000001, 0.03);
                GyroTurn(90.3, 1);
                //move towards second wobble goal
                //Wallfollower(2170, 0.7, 0.0003, 0.000001, 0.03, 73.5);
                DirectionFollower(2770, 0.7, 90.3, 0.0003, 0.000001, 0.03);
                //pick up wobble goal
                ArmClawPosition(-0.5, 600);
                hexjerry.setPower(-1);
                ArmTurn(-0.97, 480);
                //turn to face box a
                GyroTurn(28, 1);
                //place second wobble goal in box a and park on line
                DirectionFollower(3700, 1, 28, 0.0003, 0.000001, 0.03);
                ArmTurn(0.8, 930);
                ArmClawPosition(0.5, 500);
                ArmTurn(-0.6, 350);
                GyroTurn(0, 0.9);

            }

            else if (Box_B) {

                AttachmentsControl(AttachmentStates.STARTINTAKE, 0);
                AttachmentsControl(AttachmentStates.STARTSHOOTER, 0.55);
                //move towards box b
                DirectionFollower(3520, 1, 0, 0.0003, 0.000001, 0.03);
                //place wobble goal in box b
                GyroTurn(35, 0.7);
                ArmTurn(0.8, 750);
                sleep(200);
                ArmClawPosition(0.5, 600);
                ArmTurn(-0.6, 1210);
                //back up to rings and pick up rings and face high goal
                GyroTurn(-26, 0.9);
                DirectionFollower(-3540, 1, -26, 0.0003, 0.000001, 0.03);
                GyroTurn(0, 0.9);
                DirectionFollower(1080, 1, 0, 0.0003, 0.000001, 0.03);
                //shoot ring
                SetMotorPower(0);
                sleep(200);
                AttachmentsControl(AttachmentStates.LAUNCH, 0.63);
                //park on line
                DirectionFollower(1200, 1, 0, 0.0003, 0.000001, 0.03);

            }

            else if (Box_C) {

                AttachmentsControl(AttachmentStates.STARTINTAKE, 0);
                DirectionFollower(4200, 1, 0, 0.0003, 0.000001, 0.03);
                GyroTurn(45, 0.9);
                DirectionFollower(2530, 1, 45, 0.0003, 0.000001, 0.03);
                ArmTurn(0.8, 700);
                sleep(200);
                ArmClawPosition(0.5, 600);
                ArmTurn(-0.6, 1100);
                DirectionFollower(-700, 1, 45, 0.0003, 0.000001, 0.03);
                GyroTurn(-6, 0.9);
                DirectionFollower(-7100, 0.8, -6, 0.0003, 0.000001, 0.03);
                AttachmentsControl(AttachmentStates.STARTSHOOTER, 0.6);
                AttachmentsControl(AttachmentStates.STOPINTAKE, 0.6);
                DirectionFollower(2200, 1, -6, 0.0003, 0.000001, 0.03);
                SetMotorPower(0);
                sleep(500);
                AttachmentsControl(AttachmentStates.LAUNCH, 0.6);
                AttachmentsControl(AttachmentStates.LAUNCH, 0.6);
                AttachmentsControl(AttachmentStates.LAUNCH, 0.6);
                DirectionFollower(1750,1, -6, 0.0003, 0.000001, 0.03);

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
                                              DcMotor ArmJerry) {

        intakejerry = IntakeJerry;
        gatejerry = GateJerry;
        shooterjerry = ShooterJerry;
        conveyorjerry = ConveyorJerry;
        hexjerry = HexJerry;
        armjerry = ArmJerry;

    }

    private void SetMotorPower(double x) {

        frontleft.setPower(x);
        frontright.setPower(x);

    }

    private void MotorTurn(double x, double y) {

        frontleft.setPower(x);
        frontright.setPower(y);

    }

    private void ArmTurn(double power, long time) {

        armjerry.setPower(power);
        sleep(time);
        armjerry.setPower(-0.04);

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
        armjerry.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void AttachmentsControl (AttachmentStates status, double powerofshooter) {

        double difference;
        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            //shooterjerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterjerry.setPower(powerofshooter);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            shooterjerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            intakejerry.setPower(1);
            //sleep(6000);
            conveyorjerry.setPower(0.45);
            //sleep(6000);

        } else if (Status == AttachmentStates.STOPINTAKE) {

            intakejerry.setPower(0);
            conveyorjerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            difference = powerofshooter - shooterjerry.getPower();
            //shooterjerry.setPower(powerofshooter + difference);
            //sleep(200);

            if (shooterjerry.getPower() < powerofshooter) {

                while (shooterjerry.getPower() < powerofshooter) {
                    //shooterjerry.setPower(powerofshooter + 0.08);
                    shooterjerry.setPower(powerofshooter + difference + 0.01);
                }
            }
            else {

                while (shooterjerry.getPower() > powerofshooter) {
                    //shooterjerry.setPower(powerofshooter - 0.08);
                    shooterjerry.setPower(powerofshooter + difference - 0.01);
                }
            }

            intakejerry.setPower(1);
            gatejerry.setPower(0.6);
            conveyorjerry.setPower(0.6);
            sleep(700);
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

    private void MoveEncoder(double TargetPosition, double Power) {

        if (TargetPosition > 0) {

            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SetDirection(MoveDirection.FORWARD);
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