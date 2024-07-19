package org.firstinspires.ftc.teamcode.Nationala;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "TeleOP pentru copii")
public class TeleOP_Interact extends LinearOpMode {

    DcMotorEx motorDR, motorST;
    DcMotorEx lansareDR, lansareST;
    @Override
    public void runOpMode() throws InterruptedException {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        lansareDR = hardwareMap.get(DcMotorEx.class, "lansareDR");
        lansareST = hardwareMap.get(DcMotorEx.class, "lansareST");

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);
        lansareST.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lansareDR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lansareST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            if(abs(gamepad1.right_stick_y) > 0.01) {
                motorDR.setPower(gamepad1.right_stick_y);
                motorST.setPower(gamepad1.right_stick_y);
            }

            if(abs(gamepad1.left_stick_x) > 0.01) {
                motorDR.setPower(-gamepad1.left_stick_x);
                motorST.setPower(gamepad1.left_stick_x);
            }

            if(gamepad1.a) {
                lansareDR.setPower(1);
                lansareST.setPower(1);
            }

            else {
                lansareDR.setPower(0);
                lansareST.setPower(0);
            }
        }

    }
}
