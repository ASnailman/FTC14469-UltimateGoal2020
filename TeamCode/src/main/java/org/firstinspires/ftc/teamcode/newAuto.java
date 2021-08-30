package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(name = "newAuto", group = "autonomous")
public class newAuto<num_of_rings> extends LinearOpMode {

    static DcMotor LeftJerry;
    static DcMotor RightJerry;
    static DcMotor ConveyorJerry;
    static DcMotor GateJerry;
    static DcMotor ShooterJerry;
    static DcMotor IntakeJerry;
    static DcMotor HexJerry;
    static Servo ServoJerry0;
    static MoveDirection Direction;
    static AttachmentStates Status;

    double globalangle;
    double Lpower;
    double Rpower;
    double SteeringOutput;
    double current_value;
    double prev_value = 0;
    double final_value;

    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    String num_of_rings;
    boolean Box_A;
    boolean Box_B;
    boolean Box_C;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AXhILmj/////AAABmZ2G36Eg5k0ThibzOCYNI3414QDSzs2D8IZaPDmv7GTK1DM+1q2KTcH4uAQmWbMIExGi0CtO5JWf2U0nO2HyBuco2BWCXxRi+y3AKuELmddFEb2JYUOIpvTZ7MolJvUdRxhmjIo5Y4N5Vl9uk2tXXZ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {

        MotorInitialize(

                hardwareMap.dcMotor.get("LeftJerry"),
                hardwareMap.dcMotor.get("RightJerry")

        );


        HexJerry = hardwareMap.dcMotor.get("HexJerry");
        ServoJerry0 = hardwareMap.servo.get("ServoJerry0");
        ConveyorJerry = hardwareMap.dcMotor.get("ConveyorJerry");
        GateJerry = hardwareMap.dcMotor.get("GateJerry");
        ShooterJerry = hardwareMap.dcMotor.get("ShooterJerry");
        IntakeJerry = hardwareMap.dcMotor.get("IntakeJerry");


        LeftJerry.setDirection(DcMotorSimple.Direction.REVERSE);
        RightJerry.setDirection(DcMotorSimple.Direction.FORWARD);
        HexJerry.setDirection(DcMotor.Direction.FORWARD);
        ServoJerry0.setDirection(Servo.Direction.FORWARD);
        ConveyorJerry.setDirection(DcMotor.Direction.FORWARD);
        GateJerry.setDirection(DcMotor.Direction.FORWARD);
        ShooterJerry.setDirection(DcMotor.Direction.REVERSE);
        IntakeJerry.setDirection(DcMotor.Direction.REVERSE);


        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        EncoderInitialize();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        while (!isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();

        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        telemetry.update();

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        globalangle = 0;

        initVuforia();
        initTfod();


        ServoJerry0.setPosition(0);

        if (tfod != null) {
            tfod.activate();
        }


        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            MoveEncoder(1000, 0.8);
            GyroTurn(20);
            DirectionFollower(8080, 0.8, 0.003);
            ArmMethod(0.8);
            sleep(1000);
            ServoMethod(1);
            ArmMethod(-0.8);
            sleep(600);
            ArmMethod(0);
            MoveEncoder(-2600, 1);


            ET.reset();

            while (ET.milliseconds() < 2500) {
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



        }
        }

            private void EncoderInitialize() {

                LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }


            private void MotorInitialize(DcMotor front_left_motor,
                                         DcMotor front_right_motor

            ) {

                LeftJerry = front_left_motor;
                RightJerry = front_right_motor;

            }

            private void SetMotorPower(double x) {

                LeftJerry.setPower(x);
                RightJerry.setPower(x);

            }

            private void MotorTurn(double x, double y) {

                LeftJerry.setPower(x);
                RightJerry.setPower(y);

            }

            private void ArmMethod(double x){

                HexJerry.setPower(x);

            }

            private void ServoMethod(double x){

                ServoJerry0.setPosition(x);

            }


            public static void SetDirection (MoveDirection direction) {

                Direction = direction;

                if (Direction == MoveDirection.FORWARD) {

                    RightJerry.setDirection(DcMotorSimple.Direction.FORWARD);
                    LeftJerry.setDirection(DcMotorSimple.Direction.REVERSE);

                } else {

                    RightJerry.setDirection(DcMotorSimple.Direction.REVERSE);
                    LeftJerry.setDirection(DcMotorSimple.Direction.FORWARD);
                }

            }

            private double GyroContinuity() {

                orientation = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                current_value = orientation.thirdAngle;

                final_value = current_value - prev_value;

                if (final_value < -180)
                    final_value += 360;
                else if (final_value > 180)
                    final_value -= 360;

                globalangle += final_value;

                prev_value = current_value;

                return -globalangle;

            }

            private void GyroTurn (double angledegree){

                SetDirection(MoveDirection.FORWARD);

                if (angledegree > GyroContinuity()) {

                    while (GyroContinuity() <= angledegree) {

                        MotorTurn(-0.8, 0.8);

                        telemetry.addData("Gyro", GyroContinuity());
                        telemetry.update();

                    }

                    while (GyroContinuity() >= angledegree) {

                        MotorTurn(0.5, -0.5);
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

                        MotorTurn(-0.5, 0.5);
                        telemetry.addData("Gyro", GyroContinuity());
                        telemetry.update();
                    }

                }

                SetMotorPower(0);
                sleep(200);

            }

            private void MoveEncoder(double TargetPosition, double Power) {

                if (TargetPosition > 0) {

                    LeftJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SetDirection(MoveDirection.FORWARD);
                    SetMotorPower(Power);

                    while (LeftJerry.getCurrentPosition() < TargetPosition) {

                        telemetry.addData("Left Motor", LeftJerry.getCurrentPosition());
                        telemetry.update();

                    }

                    LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }
                else {

                    LeftJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SetDirection(MoveDirection.REVERSE);
                    SetMotorPower(Power);

                    while (LeftJerry.getCurrentPosition() < -TargetPosition) {

                        telemetry.addData("Left Motor", LeftJerry.getCurrentPosition());
                        telemetry.update();

                    }

                    LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                SetMotorPower(0);
                sleep(200);

            }



            private void DirectionFollower(double targetdistance, double power, double targetdirection) {

                PID PID = new PID();
                Lpower = power;
                Rpower = power;

                LeftJerry.setDirection(DcMotorSimple.Direction.REVERSE);
                RightJerry.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while (LeftJerry.getCurrentPosition() < targetdistance) {

                    SteeringOutput = PID.PID_Control(targetdirection, 0.002, 0.00000001, 0.0003, GyroContinuity());
                    Lpower = Lpower + SteeringOutput * Lpower;
                    Rpower = Rpower - SteeringOutput * Rpower;

                    LeftJerry.setPower(Lpower);
                    RightJerry.setPower(Rpower);

                    telemetry.addData("Encoder", LeftJerry.getCurrentPosition());
                    telemetry.addData("Left_Motor_Power", Lpower);
                    telemetry.addData("Right_Motor_Power", Rpower);
                    telemetry.addData("Steering", SteeringOutput);
                    telemetry.addData("DirectionZ", GyroContinuity());
                    telemetry.addData("DirectionY", orientation.secondAngle);
                    telemetry.addData("DirectionX", orientation.firstAngle);
                    telemetry.update();

                }

                LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SetMotorPower(0);
                sleep(100);
                GyroTurn(targetdirection);

            }
            private void AttachmentsControl (AttachmentStates status) {

                Status = status;

                if (Status == AttachmentStates.STARTSHOOTER) {

                    ShooterJerry.setPower(0.5);

                } else if (Status == AttachmentStates.STOPSHOOTER) {

                    ShooterJerry.setPower(0);

                }

                if (Status == AttachmentStates.STARTINTAKE) {

                    IntakeJerry.setPower(1);
                    //sleep(6000);
                    ConveyorJerry.setPower(0.5);
                    //sleep(6000);

                } else if (Status == AttachmentStates.STOPINTAKE) {

                    IntakeJerry.setPower(0);
                    ConveyorJerry.setPower(0);

                }

                if (Status == AttachmentStates.LAUNCH) {

                    IntakeJerry.setPower(1);
                    GateJerry.setPower(0.5);
                    ConveyorJerry.setPower(0.5);
                    sleep(1500);
                    IntakeJerry.setPower(0);
                    GateJerry.setPower(0);
                    ConveyorJerry.setPower(0);

                }
            }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
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

