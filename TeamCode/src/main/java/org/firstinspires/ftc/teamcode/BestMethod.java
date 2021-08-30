package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

class AdamRobot {
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static Servo Gripper;
    public static void initializationMethod(DcMotor front_left_motor,
                                            DcMotor front_right_motor,
                                            DcMotor back_left_motor,
                                            DcMotor back_right_motor,
                                            Servo back_servo)
    {

        FrontLeft = front_left_motor;
        FrontRight = front_right_motor;
        BackLeft = back_left_motor;
        BackRight = back_right_motor;
        Gripper = back_servo;


        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Gripper.setDirection(Servo.Direction.FORWARD);

    }
}
