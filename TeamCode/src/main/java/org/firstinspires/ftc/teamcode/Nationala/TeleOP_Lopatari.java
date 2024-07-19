package org.firstinspires.ftc.teamcode.Nationala;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "TeleOP_Aldea")
public class TeleOP_Lopatari extends LinearOpMode {
    DcMotorEx rightFront, rightBack, leftFront, leftBack;
    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

        //rightFront.setZeroPowerBehavior();
    }
}



