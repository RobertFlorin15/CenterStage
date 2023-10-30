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
    public DcMotorEx motor_intake = null;
    public double mod = 0;
    boolean rotire = false;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);


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
                            -gamepad1.left_stick_y*mod,
                            -(gamepad1.left_stick_x)*mod,
                            -gamepad1.right_stick_x*mod
                    )
            );

            drive.update();

            if(gamepad1.x && rotire == false) {
                rotire = true;
            }
            else if(gamepad1.x && rotire == true) {
                rotire = false;
            }
            if(rotire == true) {
                motor_intake.setPower(1);
            }
            else if(rotire == false){
                motor_intake.setPower(0);
            }
        }

    }


}
