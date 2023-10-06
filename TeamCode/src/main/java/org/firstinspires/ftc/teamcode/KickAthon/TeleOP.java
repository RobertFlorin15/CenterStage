package org.firstinspires.ftc.teamcode.KickAthon;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TeleOP_prototip")
public class TeleOP extends LinearOpMode {

    public DcMotorEx motor_avion = null;
    public DcMotorEx motor_brat = null;

    public ElapsedTime timeElapsed = new ElapsedTime() ;


    public Servo servo_avion = null;
    public Servo servo_orizontal = null;
    public Servo servo_vertical = null;

    public double mod = 0.0;
    public double viteza = 0.5;
    public int min_pos = -540;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_avion = hardwareMap.get(DcMotorEx.class, "motor_avion");
        motor_brat = hardwareMap.get(DcMotorEx.class, "motor_brat");

        servo_avion = hardwareMap.get(Servo.class, "servo_avion");
        servo_orizontal = hardwareMap.get(Servo.class, "servo_orizontal");
        servo_vertical = hardwareMap.get(Servo.class, "servo_vertical");
        servo_avion.setPosition(0.85);

        servo_vertical.setPosition(0.16);
        servo_orizontal.setPosition(0.0);

        motor_avion.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_avion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_brat.setTargetPosition(0);
        motor_brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_brat.setPower(0.8);
        motor_brat.setVelocityPIDFCoefficients(9.0, 0.0, 0.5, 1.0);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad2.right_trigger > 0.1) {
                mod = 1.0; //1
            }
            else if(gamepad2.left_trigger > 0.1){
                mod = 0.25; //0.25
            }
            else{
                mod = 0.5; //0.5
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_x*mod,
                            -(gamepad1.right_stick_y)*mod,
                            -gamepad1.left_stick_x*mod
                    )
            );

            drive.update();
            if(gamepad2.a) {
                motor_avion.setPower(1);
            }
            else{
                motor_avion.setPower(0);
            }

            if(gamepad1.x){
                servo_avion.setPosition(1);
            }

            if(timeElapsed.milliseconds() > 3.0) {
                if(gamepad2.right_bumper){
                    if(motor_brat.getTargetPosition() <= 3 && motor_brat.getTargetPosition() > min_pos-10){
                        motor_brat.setTargetPosition(motor_brat.getTargetPosition() + 5);
                    }
                }
                if(gamepad2.left_bumper){

                    if(motor_brat.getTargetPosition() <= 10 && motor_brat.getTargetPosition() > min_pos-1){
                        motor_brat.setTargetPosition(motor_brat.getTargetPosition() - 5);
                    }
                }
                timeElapsed.reset();
            }

            if(gamepad2.dpad_up){
                while(servo_vertical.getPosition() > 0.16 ){
                    servo_vertical.setPosition(servo_vertical.getPosition() - 0.0001);
                }
            }
            if(gamepad2.dpad_down){
                while (servo_vertical.getPosition() < 0.20){
                    servo_vertical.setPosition(servo_vertical.getPosition() + .0001);
                }
            }

            if(gamepad2.dpad_left){
                servo_orizontal.setPosition(0.0);
            }
            if(gamepad2.dpad_right){
                servo_orizontal.setPosition(0.21);
            }

            telemetry.addData("targ pos", motor_brat.getTargetPosition());

            telemetry.addData("curr pos", motor_brat.getCurrentPosition());
            telemetry.update();




        }

    }


}
