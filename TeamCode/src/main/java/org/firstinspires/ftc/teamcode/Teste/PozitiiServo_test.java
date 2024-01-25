package org.firstinspires.ftc.teamcode.Teste;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Identificare pozitie", group = "Test")
public class PozitiiServo_test extends LinearOpMode {
    Servo servoDR, servoST = null;
    @Override
    public void runOpMode() throws InterruptedException {
        servoDR = hardwareMap.get(Servo.class, "servoDR" );
        servoST = hardwareMap.get(Servo.class, "servoST");
        //servo.setPosition(0.7);
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("DREAPTA: ", servoDR.getPosition());
            telemetry.addData("STANGA: ", servoST.getPosition());
            telemetry.update();
        }
    }
}





