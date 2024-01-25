package org.firstinspires.ftc.teamcode.Teste;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo test", group = "Test")
public class servo_test extends LinearOpMode {
    Servo servo = null;
    double modifier = 0.0001;

    double change_modifier = 0.00000005;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servoC_ST" );
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0.0);
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a){
                servo.setPosition(servo.getPosition() + modifier);
            }
            else if(gamepad1.b){
                servo.setPosition(servo.getPosition() - modifier);
            }
            if(gamepad1.dpad_up){
                modifier += change_modifier;
            }
            else if(gamepad1.dpad_down){
                modifier -= change_modifier;
            }

            telemetry.addData("POZITIE SERVO", servo.getPosition());
            telemetry.addData("MODIFIER", modifier);
            telemetry.addData("CHANGE MODIFIER", change_modifier);

            telemetry.update();
        }
    }
}





