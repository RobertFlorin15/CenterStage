package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Test servo")
public class Test_oneservo extends LinearOpMode {
    Servo servo;
    CRServo CR7;
    DcMotorEx motor;
    double modifier = 0.0001;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "coborare_intake");

        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");

        servo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(servo.getPosition() + modifier);
            }

            if (gamepad1.b) {
                servo.setPosition(servo.getPosition() - modifier);
            }
            if (gamepad1.x){
                servo.setPosition(0.5355);
            }

            if (gamepad1.right_bumper) {
                motor.setPower(0.7);
                CR7.setPower(1);
            }
            else {
                motor.setPower(0);
                CR7.setPower(0);
            }

            telemetry.addData("Poziție: ", servo.getPosition());
            telemetry.update();
        }
    }
}
