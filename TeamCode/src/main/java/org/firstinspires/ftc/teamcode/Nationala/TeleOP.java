package org.firstinspires.ftc.teamcode.Nationala;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Nationala.Modules.BratModule;
import org.firstinspires.ftc.teamcode.Nationala.Modules.GlisieraModule;
import org.firstinspires.ftc.teamcode.Nationala.Modules.IntakeModule;
import org.firstinspires.ftc.teamcode.Nationala.Modules.SensorModule;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp (name = "TeleOP_adevărat")
public class TeleOP extends LinearOpMode {
    double mod = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GlisieraModule glisiera = new GlisieraModule(hardwareMap);
        IntakeModule intake = new IntakeModule(hardwareMap);
        BratModule brat = new BratModule(hardwareMap);
        SensorModule sensor = new SensorModule(hardwareMap);


        glisiera.init();
        intake.init();
        brat.init();
        sensor.init();


        ElapsedTime EndGame = new ElapsedTime();

        EndGame.reset();


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



            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                             gamepad1.left_stick_y * mod,
                             gamepad1.left_stick_x * mod
                    ),
                 gamepad1.right_stick_x * mod
            ));

            drive.updatePoseEstimate();

            if (gamepad2.right_trigger > 0.1) {
                intake.mananca_pixeli();
            }
            else if(gamepad2.left_trigger > 0.1){
                intake.da_la_rate_pixeli();
            }
            else {
                intake.nu_mananca_pixeli();
            }

            if (gamepad2.dpad_up) {
                intake.avion();
            }

            if (gamepad2.a) {
                brat.goUp();
            }

            if (gamepad2.b) {
                brat.goDown();
            }

            if (gamepad2.right_bumper) {
                glisiera.goUp_Ciprica();
            }

            if (gamepad2.left_bumper) {
                glisiera.goLow_Ciprica();
            }


            if (gamepad1.right_bumper) {
                glisiera.goMid();
                brat.goUp();
            }

            if (gamepad1.left_bumper) {
                glisiera.goDown();
                brat.goDown();
                brat.close();
            }
            if (gamepad1.a) {
                brat.open();
            }


            if (gamepad1.dpad_left) {
                glisiera.goLow();
            }

            if (gamepad1.dpad_right) {
                glisiera.goUp();
            }

            if (gamepad1.dpad_down) {
                glisiera.goDown_autonom();
            }

            telemetry.addData("Timp", EndGame);
            telemetry.update();
            glisiera.update();



        }
    }
}
