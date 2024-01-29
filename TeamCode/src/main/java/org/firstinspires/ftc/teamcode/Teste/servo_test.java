package org.firstinspires.ftc.teamcode.Teste;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo test", group = "Test")
public class servo_test extends LinearOpMode {
    Servo servo_extindere, servo_rotire, servoC_DR, servoC_ST;
    double modifier = 0.0001;

    double change_modifier = 0.00000005;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_extindere = hardwareMap.get(Servo.class, "servo_extindere");
        servo_rotire = hardwareMap.get(Servo.class, "servo_rotire");
        //servoC_DR = hardwareMap.get(Servo.class, "servo_avion"); //servoC_DR
        servoC_ST = hardwareMap.get(Servo.class, "servoC_ST");
        //servoC_DR.setPosition(0.79); //0.078333
        servoC_ST.setPosition(0.217);
        servo_rotire.setPosition(0.43777);
        //servo_extindere.setPosition(0.6127);
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a){
                //servo_extindere.setPosition(servo_extindere.getPosition() + modifier);
                servoC_DR.setPosition(servoC_DR.getPosition() - modifier);

            }
            else if(gamepad1.b){
                servoC_DR.setPosition(servoC_DR.getPosition() + modifier);
            }
            if(gamepad1.dpad_up){
                modifier += change_modifier;
            }
            else if(gamepad1.dpad_down){
                modifier -= change_modifier;
            }

            if(gamepad1.x) {
                //servoC_DR.setPosition(servoC_DR.getPosition() - modifier);
                servoC_ST.setPosition(servoC_ST.getPosition() - modifier);
            }
            else if (gamepad1.y) {
                //servoC_DR.setPosition(servoC_DR.getPosition() + modifier);
                servoC_ST.setPosition(servoC_ST.getPosition() + modifier);
            }

            if (gamepad1.left_bumper) {
                servo_rotire.setPosition(servo_rotire.getPosition() - modifier);
            }
            else if (gamepad1.right_bumper) {
                servo_rotire.setPosition(servo_rotire.getPosition() + modifier);
            }

            telemetry.addData("servo_extindere: ", servo_extindere.getPosition());
            telemetry.addData("servo_rotire: ", servo_rotire.getPosition());
            //telemetry.addData("servoC_DR", servoC_DR.getPosition());
            telemetry.addData("servoC_ST: ", servoC_ST.getPosition());
            telemetry.addData("MODIFIER", modifier);
            telemetry.addData("CHANGE MODIFIER", change_modifier);

            telemetry.update();
        }
    }
}





