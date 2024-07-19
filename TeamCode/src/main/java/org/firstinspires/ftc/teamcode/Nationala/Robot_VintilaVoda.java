package org.firstinspires.ftc.teamcode.Nationala;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOP_VintilaVoda")
public class Robot_VintilaVoda extends LinearOpMode {
    public DcMotorEx motor_Cirpian, motor_Costel, motor_Nadia, motor_Robert, motor_aruncare;
    public DistanceSensor Mirel;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_Cirpian = hardwareMap.get(DcMotorEx.class, "fataDR");
        motor_Costel = hardwareMap.get(DcMotorEx.class, "spateDR");
        motor_Nadia = hardwareMap.get(DcMotorEx.class, "fataST");
        motor_Robert = hardwareMap.get(DcMotorEx.class, "spateST");



        motor_Cirpian.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_Costel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_Nadia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_Robert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_Nadia.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_Robert.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_aruncare.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if(abs(gamepad1.left_stick_y) > 0.01) {
                motor_Cirpian.setPower(-gamepad1.left_stick_y*0.7);
                motor_Costel.setPower(-gamepad1.left_stick_y*0.7);
                motor_Nadia.setPower(-gamepad1.left_stick_y*0.7);
                motor_Robert.setPower(-gamepad1.left_stick_y*0.7);
            }

            else {
                motor_Cirpian.setPower(0);
                motor_Costel.setPower(0);
                motor_Nadia.setPower(0);
                motor_Robert.setPower(0);
            }

            if(gamepad1.right_stick_x > 0) {
                motor_Cirpian.setPower(-1);
                motor_Costel.setPower(-1);
                motor_Nadia.setPower(1);
                motor_Robert.setPower(1);
            }

            if(gamepad1.right_stick_x < 0) {
                motor_Cirpian.setPower(1);
                motor_Costel.setPower(1);
                motor_Nadia.setPower(-1);
                motor_Robert.setPower(-1);
            }

        }

    }
}
