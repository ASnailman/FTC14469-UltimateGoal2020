package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name = "Unused", group = "autonomous")
public class POWERSHOTprogram extends LinearOpMode {
    static DcMotor LeftJerry;
    static DcMotor RightJerry;
    static DcMotor ConveyorJerry;
    static DcMotor GateJerry;
    static DcMotor ShooterJerry;
    static DcMotor IntakeJerry;
    static DcMotor HexJerry;
    static DcMotor ArmJerry;
    static MoveDirection Direction;
    static AttachmentStates Status;
    BNO055IMU IMU1;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;
    double Lpower;
    double Rpower;
    double SteeringOutput;

    boolean boxA;
    boolean boxB;
    boolean boxC;

    String num_of_rings;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =  "AXhILmj/////AAABmZ2G36Eg5k0ThibzOCYNI3414QDSzs2D8IZaPDmv7GTK1DM+1q2KTcH4uAQmWbMIExGi0CtO5JWf2U0nO2HyBuco2BWCXxRi+y3AKuELmddFEb2JYUOIpvTZ7MolJvUdRxhmjIo5Y4N5Vl9uk2tXXZ/5NO7D0vYg/fBgpUVyO/+OnO0UIX3qotxFuCDdN86IlfygQ0p6vLtEnmUIIclVfunY4j3zDlXSbblNTMYPR96a1DjxjrNfldPEHJA+E7u8W0PvdGrtbuEqdwjgbjZjlIT30Vh/sWtaCVaY6WoqNICatk9IyHaw+Cl575F5P6tCXjR4Ib5Cr31YN3RUgLXODgKzyZM5JhRyeNjCPdHqMUqI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode () {

        MotorInitialize(

                hardwareMap.dcMotor.get("LeftJerry"),
                hardwareMap.dcMotor.get("RightJerry")

        );

        ConveyorJerry = hardwareMap.dcMotor.get("ConveyorJerry");
        GateJerry = hardwareMap.dcMotor.get("GateJerry");
        ShooterJerry = hardwareMap.dcMotor.get("ShooterJerry");
        IntakeJerry = hardwareMap.dcMotor.get("IntakeJerry");
        HexJerry = hardwareMap.dcMotor.get("HexJerry");
        ArmJerry = hardwareMap.dcMotor.get("ArmJerry");

        LeftJerry.setDirection(DcMotorSimple.Direction.REVERSE);
        RightJerry.setDirection(DcMotorSimple.Direction.FORWARD);
        ConveyorJerry.setDirection(DcMotor.Direction.REVERSE);
        GateJerry.setDirection(DcMotor.Direction.REVERSE);
        ShooterJerry.setDirection(DcMotor.Direction.FORWARD);
        IntakeJerry.setDirection(DcMotor.Direction.FORWARD);
        HexJerry.setDirection(DcMotor.Direction.REVERSE);
        ArmJerry.setDirection(DcMotor.Direction.FORWARD);


        IMU1 = hardwareMap.get(BNO055IMU.class, "imu1");
        EncoderInitialize();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU1.initialize(parameters);

        while (!isStopRequested() && !IMU1.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", IMU1.getCalibrationStatus().toString());
        telemetry.update();

        orientation = IMU1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        globalangle = 0;

        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
        }

        HexJerry.setPower(-0.5);

        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            AttachmentsControl(AttachmentStates.STARTSHOOTER);
            DirectionFollower(520, 0.8,  0, 0.0003, 0.000001, 0.03);
            GyroTurn(4, 0.30);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.LAUNCH);
            AttachmentsControl(AttachmentStates.STOPSHOOTER);
            GyroTurn(0, 0.45);
            DirectionFollower(2450, 0.8, 0, 0.0003, 0.000001, 0.03);
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

            sleep(20000);
            if (num_of_rings == "Quad") {
                boxC = true;
            } else if (num_of_rings == "Single") {
                boxB = true;
            } else {
                boxA = true;
            }
            if (boxA) {

                DirectionFollower(3800, 0.9, 0, 0.0003, 0.000001, 0.03);
                GyroTurn(90, 0.45);
                DirectionFollower(2800, 0.9, 0, 0.0003, 0.000001, 0.03);
                ArmMethod(-0.8);
                sleep(900);
                ServoMethod(0.7);
                sleep(600);
                ArmMethod(0);
                ServoMethod(0);
                GyroTurn(0, 0.45);

            } else if (boxB) {
                MoveEncoder(-1000, 0.8);
                GyroTurn(24, 0.45);
                DirectionFollower(8080, 0.7, 24, 0.0003, 0.000001, 0.03);
                ArmMethod(0.8);
                sleep(1100);
                ServoMethod(1);
                ArmMethod(-0.8);
                sleep(600);
                ArmMethod(0);
                MoveEncoder(-2730, 1);

            } else if (boxC) {
                MoveEncoder(-1000, 0.8);
                GyroTurn(22, 0.45);
                DirectionFollower(6500, 0.9, 20, 0.0003, 0.000001, 0.03);
                GyroTurn(240, 0.45);
                MoveEncoder(-600, 0.8);
                ArmMethod(0.8);
                sleep(900);
                ServoMethod(1);
                ArmMethod(-0.8);
                sleep(600);
                ArmMethod(0);
                //while (WhiteDetector() != 1) {
                //    SetMotorPower(1);
                //}
                //SetMotorPower(0);
                MoveEncoder(2000, 1);
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            break;
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

            ArmJerry.setPower(x);

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

            orientation = IMU1.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

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

        private void GyroTurn (double angledegree, double power){

            SetDirection(MoveDirection.FORWARD);

            if (angledegree > GyroContinuity()) {

                while (GyroContinuity() <= angledegree) {

                    MotorTurn(-power, power);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }

                while (GyroContinuity() >= angledegree){
                    MotorTurn(0.3, -0.3);
                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();
                }

            } else {

                while (GyroContinuity() >= angledegree) {

                    MotorTurn(power, -power);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
                while (GyroContinuity() <= angledegree) {
                    MotorTurn(-0.3, 0.3);
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



    private void DirectionFollower(double targetdistance, double power, double TargetDirection,
                                   double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        Lpower = power;
        Rpower = power;

        //frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontleft.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftJerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (LeftJerry.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower - SteeringOutput * Lpower;
            Rpower = Rpower + SteeringOutput * Rpower;

            LeftJerry.setPower(Lpower);
            RightJerry.setPower(Rpower);

            telemetry.addData("Encoder", LeftJerry.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-LeftJerry.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.REVERSE);
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0003, 0.000001, 0.03, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            Lpower = Lpower + SteeringOutput * Lpower;
            Rpower = Rpower - SteeringOutput * Rpower;

            LeftJerry.setPower(Lpower);
            RightJerry.setPower(Rpower);

            telemetry.addData("Encoder", -LeftJerry.getCurrentPosition());
            telemetry.addData("Left_Motor_Power", Lpower);
            telemetry.addData("Right_Motor_Power", Rpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        LeftJerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);
        GyroTurn(TargetDirection, 0.3);

    }

    private void AttachmentsControl (AttachmentStates status) {

        Status = status;

        if (Status == AttachmentStates.STARTSHOOTER) {

            ShooterJerry.setPower(0.69);



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
            sleep(1010);
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
