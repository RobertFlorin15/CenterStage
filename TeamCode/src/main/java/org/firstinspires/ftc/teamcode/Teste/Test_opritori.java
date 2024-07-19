package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Test_opritori")
public class Test_opritori extends LinearOpMode {
    Servo servo;
    CRServo CR7;
    DcMotorEx motor;

    //Servo opritoare_sus;

    Servo opritoare_jos;
    double modifier = 0.0001;
    //avion=0.606
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "coborare_intake");
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        //opritoare_sus = hardwareMap.get(Servo.class, "opritoare_sus");
        opritoare_jos = hardwareMap.get(Servo.class, "opritoare_jos");

        //opritoare_sus.setPosition(0);
        opritoare_jos.setPosition(0);
        //pozitie opritoare_sus 0.234
        //pozitie opritoare_joa 0.116
        servo.setPosition(0.1044);
        //BratModule bratModule = new BratModule(hardwareMap);
        //GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        //glisieraModule.init();
        //bratModule.init();


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

            if(gamepad1.right_trigger > 0.1) {
                motor.setPower(1);
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
            if(gamepad1.y) {
                opritoare_jos.setPosition(opritoare_jos.getPosition() + modifier);
            }
            if(gamepad1.x) {
                opritoare_jos.setPosition(opritoare_jos.getPosition() - modifier);
            }


            //if(gamepad1.x) {
            //  avion.setPosition(0.606);
            //}

            telemetry.addData("Poziție: " , servo.getPosition());
            telemetry.addData("Poziție: " , opritoare_jos.getPosition());
            telemetry.update();
        }
    }
}
