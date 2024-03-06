package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp (name = "Test servo")
public class Test_oneservo extends LinearOpMode {
    Servo servo;
    double modifier = 0.0001;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "rotire_cuva");

        servo.setPosition(1);

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

            telemetry.addData("Poziție: ", servo.getPosition());
            telemetry.update();
        }
    }
}
