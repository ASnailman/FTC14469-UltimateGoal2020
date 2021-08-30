package org.firstinspires.ftc.teamcode;

public enum AttachmentStates {


        STARTSHOOTER(1), STOPSHOOTER(2), STARTINTAKE(3), STOPINTAKE(4), LAUNCH(5), STOPLAUNCH(6), LAUNCHPREP(7);

        int CurrentAttachmentState;

        AttachmentStates(int CMD) {

            CurrentAttachmentState = CMD;

        }
    }

