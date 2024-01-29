package org.firstinspires.ftc.teamcode.Teste;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Identificare pozitie", group = "Test")
public class PozitiiServo_test extends LinearOpMode {
    public Servo servoC_DR, servoC_ST, servo_rotire, servo_extindere, servo_DR, servo_ST;
    @Override
    public void runOpMode() throws InterruptedException {
        servo_DR = hardwareMap.get(Servo.class, "servo_DR");
        servo_ST = hardwareMap.get(Servo.class, "servo_ST");
        servoC_DR = hardwareMap.get(Servo.class, "servoC_DR");
        servoC_ST = hardwareMap.get(Servo.class, "servoC_ST");
        servo_extindere = hardwareMap.get(Servo.class, "servo_extindere");
        servo_rotire = hardwareMap.get(Servo.class, "servo_rotire");
        //servo.setPosition(0.7);
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("servo_DR: ", servo_DR.getPosition());
            telemetry.addData("servo_ST: ", servo_ST.getPosition());
            telemetry.addData("servo_rotire: ", servo_rotire.getPosition());
            telemetry.addData("servo_extindere: ", servo_extindere.getPosition());

            telemetry.update();
        }
    }
}





