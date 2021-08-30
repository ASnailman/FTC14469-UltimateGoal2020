package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TheyAutoWin", group = "autonomous")
public class AutonomousRunUG extends LinearOpMode {

    /**private Servo Gripper;*/

    public void runOpMode() {

        MyRobot.MotorInitialize(

                hardwareMap.dcMotor.get("LeftJerry"),
                hardwareMap.dcMotor.get("RightJerry")

        );

        MyRobotAttachment.AttachmentInitialize(

                hardwareMap.dcMotor.get("GateJerry"),
                hardwareMap.dcMotor.get("ShooterJerry"),
                hardwareMap.dcMotor.get("ConveyorJerry"),
                hardwareMap.dcMotor.get("IntakeJerry")

        );

        /**Gripper = hardwareMap.servo.get("Gripper");*/
        /**Gripper.setDirection(Servo.Direction.FORWARD);*/
        MyRobot.SetDirection();
        MyRobotAttachment.AttachmentSetDirection();

        waitForStart();

        //Go forward for 8.409 seconds to reach box b

        MyRobot.SetMotorPowerUG(1);

        sleep(6000);

        MyRobot.SetMotorPower(0);

        //Moving servo to to place wobble goal in box b

        /**Gripper.setPosition(1);*/
        /**sleep(2000);*/
        /**Gripper.setPosition(0);*/

        //Run shooting mechanism at 75% power until towards the end of the run
        //Run intake mechanism at 75% power until towards the end of the run

        MyRobotAttachment.RobotShooter(0.6);
        MyRobotAttachment.RobotIntake(0.6);

        //Go backward for 3.737 seconds by setting power to negative

        MyRobot.SetMotorPowerUG(-1);

        sleep(2300);

        MyRobot.SetMotorPower(0);

        //Adjust robot to face next power shot

        MyRobot.MotorTurn(-0.533, 0.5);
        sleep(90);
        MyRobot.SetMotorPower(0);

        //Shoot First Ring

        MyRobotAttachment.RobotGate(-1);
        sleep(1500);
        MyRobotAttachment.RobotGate(0);

        //mechanism to complete power shot
        //Repeated 2 times using for loop to shoot rings

            for (int i = 0; i < 1; i++) {
                //Set Conveyor to 80% power and run for 1.7 second (1700ms) to put ring in shooting
                MyRobotAttachment.RobotConveyor(-1);
                sleep(1500);
                MyRobotAttachment.RobotConveyor(0);
                //Shoot Ring
                MyRobotAttachment.RobotGate(-1);
                sleep(1500);
                MyRobotAttachment.RobotGate(0);
                //Turn robot to adjust position
                MyRobot.MotorTurn(-0.533, 0.5);
                sleep(90);
                MyRobot.SetMotorPower(0);
            }

        MyRobotAttachment.RobotGate(-1);
        //Set Conveyor to 80% power and run for 1.7 second (1700ms) to put ring in shooting
        MyRobotAttachment.RobotConveyor(-1);
        sleep(6000);
        MyRobotAttachment.RobotConveyor(0);

        //Set all powers of unused motors to 0

        MyRobotAttachment.RobotShooter(0);
        MyRobotAttachment.RobotIntake(0);
        MyRobotAttachment.RobotGate(0);

        //Park on white line

        MyRobot.MotorTurn(0.533, -0.5);
        sleep(270);
        MyRobot.SetMotorPower(0);

        MyRobot.SetMotorPowerUG(1);
        sleep(1350);
        MyRobot.SetMotorPower(0);

    }

}


