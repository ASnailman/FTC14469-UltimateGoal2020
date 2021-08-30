package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MyRobot {

    static DcMotor frontleft;
    static DcMotor frontright;
    //static DcMotor backleft;
    //static DcMotor backright;
    static MoveDirection Direction;

    public static void MotorInitialize (DcMotor LeftJerry,
                                        DcMotor RightJerry) {

        frontleft = LeftJerry;
        frontright = RightJerry;
        //backleft = back_left_motor;
        //backright = back_right_motor;

    }

    public static void SetDirection () {

            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleft.setDirection(DcMotorSimple.Direction.FORWARD);

        }

    public static void SetMotorPower (double x) {

        frontleft.setPower(x);
        frontright.setPower(x);
        //backleft.setPower(x);
        //backright.setPower(x);

    }

    public static void SetMotorPowerUG (double x) {

        frontleft.setPower(0.535 * x);
        frontright.setPower(0.5 * x);

    }

    public static void MotorTurn (double x, double y) {

        frontleft.setPower(x);
        frontright.setPower(y);
        //backleft.setPower(x);
        //backright.setPower(y);

    }

}
