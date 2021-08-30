package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "JerryAutoWin", group = "autonomous")
public class JerryRobot extends LinearOpMode {

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
        //colorsensor = hardwareMap.get(ColorSensor.class, "ColorJerry1");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "ColorJerry1");

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

        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            //DirectionFollower(4250, 0.9, 0);
            //GyroTurn(90);
            //DirectionFollower(1000, 0.9, 90);
            //AttachmentsControl(AttachmentStates.STARTSHOOTER);
            //GyroTurn(-8);
            //AttachmentsControl(AttachmentStates.LAUNCH);
            //AttachmentsControl(AttachmentStates.LAUNCH);
            //AttachmentsControl(AttachmentStates.LAUNCH);
            //AttachmentsControl(AttachmentStates.STOPSHOOTER);
            //GyroTurn(50);
            //MoveEncoder(-2600, 0.7);
            //GyroTurn(93);
            //MoveEncoder(-500, 0.6);

            AttachmentsControl(AttachmentStates.STARTSHOOTER);
            DirectionFollower(4100, 0.9, 0, 0.003, 0.00001, 0.0003);
            GyroTurn(12);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.STOPSHOOTER);
            GyroTurn(25);
            MoveEncoder(-1200, 0.8);
            GyroTurn(84);
            MoveEncoder(-550, 0.8);

            ET.reset();
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

                GyroTurn(0);
                DirectionFollower(2200, 0.9, 0, 0.0003, 0.000001, 0.03);
                GyroTurn(-90);
                MoveEncoder(-1000, 0.8);
                ArmTurn(0.8);
                sleep(900);
                ClawPosition(1);
                ArmTurn(-0.8);
                sleep(300);
                ArmTurn(0);
                GyroTurn(0);

            } else if (Box_B) {

                //MoveEncoder(-1000, 0.85);
                //GyroTurn(0);
                //DirectionFollower(3500, 0.85, 0);
                //GyroTurn(-90);
                //SetMotorPower(0);
                //sleep(3000);
                //GyroTurn(0);

                MoveEncoder(-1000, 0.8);
                GyroTurn(30);
                DirectionFollower(8500, 0.7, 24, 0.0003, 0.000001, 0.03);
                ArmTurn(0.8);
                sleep(900);
                ArmTurn(0);
                sleep(500);
                ClawPosition(1);
                ArmTurn(-0.8);
                sleep(300);
                ArmTurn(0);
                //while (WhiteDetector() != 1) {
                //   SetMotorPower(1);
                //}
                //SetMotorPower(0);
                GyroTurn(45);
                MoveEncoder(-2500, 1);

            } else if (Box_C) {

                MoveEncoder(-1000, 0.8);
                GyroTurn(22);
                DirectionFollower(6500, 0.9, 22, 0.0003, 0.000001, 0.03);
                GyroTurn(240);
                MoveEncoder(-600, 0.8);
                ArmTurn(0.8);
                sleep(900);
                ClawPosition(1);
                ArmTurn(-0.8);
                sleep(300);
                ArmTurn(0);
                //while (WhiteDetector() != 1) {
                //    SetMotorPower(1);
                //}
                //SetMotorPower(0);
                MoveEncoder(2300, 1);
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

    private void ArmTurn(double x) {

        hexjerry.setPower(x);

    }

    private void ClawPosition(double x) {

        armjerry.setPower(x);

    }

    private static void SetDirection (MoveDirection direction) {

        Direction = direction;

        if (Direction == MoveDirection.FORWARD) {

            frontright.setDirection(DcMotorSimple.Direction.FORWARD);
            frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        } else {

            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleft.setDirection(DcMotorSimple.Direction.FORWARD);

        }
    }

    private void AttachmentSetDirection () {

        shooterjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorjerry.setDirection(DcMotorSimple.Direction.FORWARD);
        gatejerry.setDirection(DcMotorSimple.Direction.FORWARD);
        intakejerry.setDirection(DcMotorSimple.Direction.REVERSE);
        hexjerry.setDirection(DcMotorSimple.Direction.FORWARD);
        armjerry.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    private void AttachmentsControl (AttachmentStates status) {

        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            shooterjerry.setPower(0.5985);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            shooterjerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            intakejerry.setPower(1);
            //sleep(6000);
            conveyorjerry.setPower(0.5);
            //sleep(6000);

        } else if (Status == AttachmentStates.STOPINTAKE) {

            intakejerry.setPower(0);
            conveyorjerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            intakejerry.setPower(1);
            gatejerry.setPower(0.5);
            conveyorjerry.setPower(0.5);
            sleep(1300);
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

    private void GyroTurn (double angledegree) {

        SetDirection(MoveDirection.FORWARD);

        if (angledegree > GyroContinuity()) {

            while (GyroContinuity() <= angledegree) {

                MotorTurn(-0.8, 0.8);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            while (GyroContinuity() >= angledegree) {

                MotorTurn(0.45, -0.45);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

        } else {

            while (GyroContinuity() >= angledegree) {

                MotorTurn(0.8, -0.8);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            while (GyroContinuity() <= angledegree) {

                MotorTurn(-0.45, 0.45);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }
        }
        SetMotorPower(0);
        sleep(200);
    }

    private void MoveEncoder(double TargetPosition, double Power) {

        if (TargetPosition > 0) {

            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SetDirection(MoveDirection.FORWARD);
            SetMotorPower(Power);

            while (frontleft.getCurrentPosition() < TargetPosition) {

                telemetry.addData("Left Motor", frontleft.getCurrentPosition());
                telemetry.update();

            }

            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        else {

            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SetDirection(MoveDirection.REVERSE);
            SetMotorPower(Power);

            while (frontleft.getCurrentPosition() < -TargetPosition) {

                telemetry.addData("Left Motor", frontleft.getCurrentPosition());
                telemetry.update();

            }

            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        SetMotorPower(0);
        sleep(200);

    }

    private static void EncoderInitialize() {

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void DirectionFollower(double targetdistance, double power, double TargetDirection,
                                   double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        //frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (frontleft.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", frontleft.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-frontleft.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            SteeringOutput = PID.PID_Control(TargetDirection, 0.0000001, 0.00000000000001, 0.00000003, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            frontleft.setPower(Lpower);
            frontright.setPower(Rpower);

            telemetry.addData("Encoder", -frontleft.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);
        GyroTurn(TargetDirection);

    }

    private int WhiteDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addLine()
                .addData("H", "%.3f", HSV[0])
                .addData("S", "%.3f", HSV[1])
                .addData("V", "%.3f", HSV[2]);
        telemetry.update();

        int White = 1;
        int Unkwown = 0;

        if (HSV[1] <= 3) {
            if (HSV[2] >= 96) {
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