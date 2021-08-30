package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "AdamAut", group = "AdamGroup")
 public class BestTeleOp extends LinearOpMode {
    DcMotor LeftJerry;
    DcMotor RightJerry;
    DcMotor ConveyorJerry;
    DcMotor GateJerry;
    DcMotor ShooterJerry;
    DcMotor IntakeJerry;

    public void runOpMode () throws InterruptedException {


       // MyRobot.initializationMethod(
           //     hardwareMap.dcMotor.get("FrontLeft"),
            //    hardwareMap.dcMotor.get("FrontRight"),
          //      hardwareMap.dcMotor.get("BackLeft"),
            //    hardwareMap.dcMotor.get("BackRight"),
        //        hardwareMap.servo.get("Gripper")
      //  );

        LeftJerry = hardwareMap.dcMotor.get("LeftJerry");
        RightJerry =   hardwareMap.dcMotor.get("RightJerry");
               ConveyorJerry = hardwareMap.dcMotor.get("ConveyorJerry");
        GateJerry = hardwareMap.dcMotor.get("GateJerry");
        ShooterJerry = hardwareMap.dcMotor.get("ShooterJerry");
        IntakeJerry = hardwareMap.dcMotor.get("IntakeJerry");

        LeftJerry.setDirection(DcMotorSimple.Direction.FORWARD);
        RightJerry.setDirection(DcMotorSimple.Direction.REVERSE);
        ConveyorJerry.setDirection(DcMotor.Direction.REVERSE);
        GateJerry.setDirection(DcMotor.Direction.FORWARD);
        ShooterJerry.setDirection(DcMotor.Direction.FORWARD);
        IntakeJerry.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        LeftJerry.setPower(0.535);
        RightJerry.setPower(0.5);
        sleep(6000);

        LeftJerry.setPower(0);
        RightJerry.setPower(0);
        sleep(1000);

        LeftJerry.setPower(-0.535);
        RightJerry.setPower(-0.5);
        sleep(2406);

        LeftJerry.setPower(0);
        RightJerry.setPower(0);
        sleep(1150);

        LeftJerry.setPower(0.535);
        RightJerry.setPower(-0.5);
        sleep(55);


        LeftJerry.setPower(0);
        RightJerry.setPower(0);
        sleep(30);


        IntakeJerry.setPower(1);
        ShooterJerry.setPower(0.6215);
        sleep(2000);
        ConveyorJerry.setPower(-1);
        GateJerry.setPower(-1);
        sleep(1500);

        ConveyorJerry.setPower(0);
        GateJerry.setPower(0);
        sleep(1000);

        ConveyorJerry.setPower(-1);
        GateJerry.setPower(-1);
        sleep(2000);

        ConveyorJerry.setPower(0);
        GateJerry.setPower(0);
        sleep(1000);

        ConveyorJerry.setPower(-1);
        GateJerry.setPower(-1);
        sleep(6000);

        LeftJerry.setPower(-0.535);
        RightJerry.setPower(0.5);
        sleep(55);

        LeftJerry.setPower(0);
        RightJerry.setPower(0);
        sleep(1450);


        LeftJerry.setPower(0.535);
        RightJerry.setPower(0.5);
        sleep(1145);
    }
}
