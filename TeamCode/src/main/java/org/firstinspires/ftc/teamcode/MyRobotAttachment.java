package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MyRobotAttachment {

    static DcMotor gatejerry;
    static DcMotor shooterjerry;
    static DcMotor conveyorjerry;
    static DcMotor intakejerry;

    public static void AttachmentInitialize (DcMotor GateJerry,
                                             DcMotor ShooterJerry,
                                             DcMotor ConveyorJerry,
                                             DcMotor IntakeJerry) {

        intakejerry = IntakeJerry;
        gatejerry = GateJerry;
        shooterjerry = ShooterJerry;
        conveyorjerry = ConveyorJerry;

    }

    public static void AttachmentSetDirection () {

        shooterjerry.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorjerry.setDirection(DcMotorSimple.Direction.REVERSE);
        gatejerry.setDirection(DcMotorSimple.Direction.FORWARD);
        intakejerry.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static void RobotGate (double x) {

        gatejerry.setPower(x);

    }

    public static void RobotShooter (double x) {

        shooterjerry.setPower(x);

    }

    public static void RobotConveyor (double x) {

        conveyorjerry.setPower(x);

    }

    public static void RobotIntake (double x) {

        intakejerry.setPower(x);

    }

}
