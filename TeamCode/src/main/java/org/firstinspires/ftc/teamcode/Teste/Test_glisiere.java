package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Test GLisiere")
public class Test_glisiere extends LinearOpMode {
    public DcMotorEx motorDR, motorST;

    double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        motorDR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDR.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            if(gamepad1.a) {
                motorDR.setPower(power);
                motorST.setPower(power);
            }

            else if(gamepad1.b){
                motorDR.setPower(-power);
                motorST.setPower(-power);
            }
            else{
                motorDR.setPower(0);
                motorST.setPower(0);
            }

            if(gamepad1.dpad_up) {
                power+=0.00001;
            }
            else if(gamepad1.dpad_down) {
                power-=0.00001;
            }

            if(gamepad1.left_bumper){
                motorST.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDR.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(gamepad1.right_bumper) {
                motorST.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDR.setDirection(DcMotorSimple.Direction.FORWARD);
            }



            telemetry.addData("POWER: ", power);
            telemetry.addData("motorDR: ", motorDR.getDirection());
            telemetry.addData("motorST: ", motorST.getDirection());

            telemetry.update();

        }

    }
}
