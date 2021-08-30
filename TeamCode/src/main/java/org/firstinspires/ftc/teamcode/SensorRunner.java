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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SensorTestingTeleOp", group="Linear Opmode")

public class SensorRunner extends LinearOpMode {

    static DcMotor frontleft;
    static DcMotor frontright;
    static DcMotor gatejerry;
    static DcMotor shooterjerry;
    static DcMotor conveyorjerry;
    static DcMotor intakejerry;
    static DcMotor hexjerry;
    static DcMotor armjerry;
    static Servo servojerry;
    static MoveDirection Direction;
    NormalizedColorSensor colorsensor;
    DistanceSensor distancesensor;
    static AttachmentStates Status;
    double power;
    double loading_speed;
    double speed;
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


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        AttachmentSetDirection();

        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "ColorJerry1");
        distancesensor = hardwareMap.get(DistanceSensor.class,"DistanceJerry1");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU = hardwareMap.get(BNO055IMU.class, "imu1");
        IMU.initialize(parameters);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        telemetry.update();

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

        shooterjerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        power = 0.66;
        loading_speed = 0.6;
        boolean button_a_already_pressed = false;
        boolean button_b_already_pressed = false;
        boolean button_x_already_pressed = false;
        boolean button_cross_already_pressed = false;
        boolean button_circle_already_pressed = false;
        boolean button_dpad_down_already_pressed = false;
        boolean button_dpad_up_already_pressed = false;
        boolean lower_arm = true;

        ElapsedTime ET = new ElapsedTime();

        double checkInterval = 0.1;
        double prevPosition = shooterjerry.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                if (gamepad2.dpad_left) {
                    WhiteDetector();
                }

                if (gamepad2.dpad_right) {
                    Distancesensor();
                }

            }
        }
    }

    private void MotorInitialize (DcMotor LeftJerry,
                                 DcMotor RightJerry

    ) {

        frontleft = LeftJerry;
        frontright = RightJerry;
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void ArmClawPosition(double power, long time) {

        hexjerry.setPower(power);
        sleep(time);
        hexjerry.setPower(0);

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

    }

    private void AttachmentsControl (AttachmentStates status, double powerofshooter, long amount) {

        Status = status;
        //double difference;

        if (Status == AttachmentStates.STARTSHOOTER) {

            //shooterjerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterjerry.setPower(powerofshooter);

        } else if (Status == AttachmentStates.STOPSHOOTER) {

            shooterjerry.setPower(0);

        }

        if (Status == AttachmentStates.STARTINTAKE) {

            intakejerry.setPower(1);
            //sleep(6000);
            //conveyorjerry.setPower(0.45);
            conveyorjerry.setPower(0.8);
            //sleep(6000);

        } else if (Status == AttachmentStates.STOPINTAKE) {

            intakejerry.setPower(0);
            conveyorjerry.setPower(0);
            gatejerry.setPower(0);

        }

        if (Status == AttachmentStates.LAUNCH) {

            //difference = powerofshooter - shooterjerry.getPower();
            //shooterjerry.setPower(powerofshooter + difference);
            //sleep(200);

            //if (shooterjerry.getPower() < powerofshooter) {

            //while (shooterjerry.getPower() < powerofshooter) {
            //shooterjerry.setPower(powerofshooter + 0.08);
            //shooterjerry.setPower(powerofshooter + difference + 0.01);

            //}
            //}
            //else {

            //while (shooterjerry.getPower() > powerofshooter) {
            //shooterjerry.setPower(powerofshooter - 0.08);
            //shooterjerry.setPower(powerofshooter + difference - 0.01);
            //}
            //}

            intakejerry.setPower(1);
            gatejerry.setPower(0.55);
            conveyorjerry.setPower(0.55);
            sleep(amount);

        }

        if (Status == AttachmentStates.STOPLAUNCH) {

            intakejerry.setPower(0);
            gatejerry.setPower(0);
            conveyorjerry.setPower(0);

        }
    }

    private void SetMotorPower(double x) {

        frontleft.setPower(x);
        frontright.setPower(x);

    }

    private void ArmTurn_RTP(int arm_pos_chg) {

        //armjerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armjerry.setTargetPosition(arm_pos_chg);
        armjerry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armjerry.setPower(0.7);

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
                return White;
            } else {
                telemetry.addData("Color:", "Unknown");
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            return Unkwown;
        }
    }

    private int RedDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(70);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int Red = 1;
        int Unkwown = 0;

        if (HSV[1] <= 0.62) {
            if (HSV[2] >= 0.37) {
                telemetry.addData("Color:", "Red");
                SetMotorPower(0);
                return Red;
            } else {
                telemetry.addData("Color:", "Unknown");
                SetMotorPower(0.6);
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            SetMotorPower(0.6);
            return Unkwown;
        }
    }

    private void Distancesensor() {

        telemetry.addData("Distance From Wall", distancesensor.getDistance(DistanceUnit.CM));
        telemetry.update();

    }

    private void MotorTurn(double x, double y) {

        frontleft.setPower(x);
        frontright.setPower(y);

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

}

