package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name= "Test_senzori_fct_de_ciprica" )
public class Test_senzori extends LinearOpMode {

    public Servo opritoare_sus , opritoare_jos;

    ColorRangeSensor sensor1,sensor2;
    public CRServo CR7;

    public DcMotorEx motor_intake;
    @Override
    public void runOpMode() throws InterruptedException {

        sensor1 = hardwareMap.get(ColorRangeSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorRangeSensor.class, "sensor2");
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        opritoare_sus = hardwareMap.get(Servo.class, "opritoare_sus");
        opritoare_jos = hardwareMap.get(Servo.class, "opritoare_jos");

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        opritoare_sus.setPosition(0.243);
        opritoare_jos.setPosition(0);
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.right_trigger > 0.1) {
                motor_intake.setPower(1);
                CR7.setPower(1);
            }
            else if(gamepad1.left_trigger > 0.1) {
                motor_intake.setPower(-1);
                CR7.setPower(-1);
            }
            else {
                motor_intake.setPower(0);
                CR7.setPower(0);
            }
            if( sensor1.getDistance(DistanceUnit.CM) < 1.6 && sensor2.getDistance(DistanceUnit.CM) < 1.6 ) {
                opritoare_sus.setPosition(0);
                opritoare_jos.setPosition(0.116);
            }
            else {
                opritoare_sus.setPosition(0.243);
                opritoare_jos.setPosition(0);
            }

            telemetry.addData("sensor1: ", sensor1.getDistance(DistanceUnit.CM));
            telemetry.addData("sensor2: ", sensor2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
