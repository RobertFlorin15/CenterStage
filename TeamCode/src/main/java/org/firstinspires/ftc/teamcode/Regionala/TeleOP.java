package org.firstinspires.ftc.teamcode.Regionala;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Regionala.Modules.BratModule;
import org.firstinspires.ftc.teamcode.Regionala.Modules.GlisieraModule;
import org.firstinspires.ftc.teamcode.Regionala.Modules.IntakeModule;
import org.firstinspires.ftc.teamcode.Regionala.Modules.SensorModule;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOP_adevărat")
public class TeleOP extends LinearOpMode {
    double mod;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GlisieraModule glisiera = new GlisieraModule(hardwareMap);
        IntakeModule intake = new IntakeModule(hardwareMap);
        BratModule brat = new BratModule(hardwareMap);
        SensorModule sensor = new SensorModule(hardwareMap);


        glisiera.init();
        intake.init();
        brat.init();
        sensor.init();


        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0.1) {
                mod = 1.0;
            }
            else if(gamepad1.left_trigger > 0.1) {
                mod = 0.25;
            }
            else {
                mod = 0.5;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            - gamepad1.left_stick_y * mod,
                            - gamepad1.left_stick_x * mod,
                            - gamepad1.right_stick_x * mod
                    )
            );

            drive.update();

            if (gamepad1.x) {
                intake.mananca_pixeli();
            }
            else {
                intake.nu_mananca_pixeli();
            }

            if (gamepad1.right_bumper) {
                glisiera.goUp();
                brat.goUp();
            }

            if (gamepad1.left_bumper) {
                glisiera.goDown();
                brat.goDown();
                brat.close();
            }
            if(gamepad1.a){
                intake.da_la_rate_pixeli();
            }


            if (gamepad1.dpad_up) {
                brat.open();
            }



            glisiera.update();

        }
    }
}
