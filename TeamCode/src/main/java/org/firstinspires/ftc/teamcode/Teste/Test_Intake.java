package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp (name = "INTAKE BUCLUCAS DACA BUBUIE MA OMOR")
public class Test_Intake extends LinearOpMode {
    public DcMotorEx motor;
    public CRServo CR7;
    public Servo intake_inchidere;
    public double modifier = 0.000001, power = 0.5;
    ColorRangeSensor sensor1, sensor2;
    int pixeli = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        sensor1 = hardwareMap.get(ColorRangeSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorRangeSensor.class, "sensor2");
        intake_inchidere = hardwareMap.get(Servo.class, "intake_inchidere");

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1);
                CR7.setPower(1);
            }

            else {
                motor.setPower(0);
                CR7.setPower(0);
            }

            if (gamepad1.dpad_up) {
                power+=modifier;
            }

            else if (gamepad1.dpad_down) {
                power-=modifier;
            }

            if (gamepad1.right_bumper) {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                CR7.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            else if (gamepad1.left_bumper) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                CR7.setDirection(DcMotorSimple.Direction.FORWARD);
            }


            if (sensor1.getDistance(DistanceUnit.CM) < 1.5 && sensor2.getDistance(DistanceUnit.CM) < 1.5) {
                intake_inchidere.setPosition(0.265);
            }

            else {
                intake_inchidere.setPosition(0);
            }

            telemetry.addData("Directie: ", motor.getDirection());
            telemetry.update();

        }
    }
}