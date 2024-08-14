package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Test servo")
public class Test_oneservo extends LinearOpMode {
    Servo servo, opritor;
    CRServo CR7;
    DcMotorEx motor;
    double modifier = 0.0001;
    //avion=0.62
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servoST");
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        opritor = hardwareMap.get(Servo.class, "opritoare_sus");

        servo.setPosition(0);
        //BratModule bratModule = new BratModule(hardwareMap);
        //GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        //glisieraModule.init();
        //bratModule.init();

        opritor.setPosition(0.234);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo.setPosition(servo.getPosition() + modifier);

            }

            if(gamepad1.b) {
                servo.setPosition(servo.getPosition() - modifier);
            }

            /*if(gamepad1.right_bumper) {
                glisieraModule.goMid();
                bratModule.goUp();

            }

            if(gamepad1.left_bumper) {
                glisieraModule.goDown();
                bratModule.goDown();
            }*/
            if(gamepad1.x) {
                servo.setPosition(0);
            }
            if(gamepad1.right_trigger > 0.1) {
                motor.setPower(0.77);
                CR7.setPower(1);
            }
            else if(gamepad1.left_trigger > 0.1) {
                motor.setPower(-1);
                CR7.setPower(-1);
            }
            else {
                motor.setPower(0);
                CR7.setPower(0);
            }


            //if(gamepad1.x) {
              //  avion.setPosition(0.606);
            //}

            telemetry.addData("Poziție: " , servo.getPosition());
            telemetry.update();
        }
    }
}
