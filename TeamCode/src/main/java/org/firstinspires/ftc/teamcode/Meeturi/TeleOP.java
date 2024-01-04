package org.firstinspires.ftc.teamcode.Meeturi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Meeturi.Module.GlisieraModule;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp(name="TeleOP")
public class TeleOP extends LinearOpMode {
    public DcMotorEx motor_intake, motor_lift;
    public DcMotorEx motorDR_ENC, motorST;
    public Servo stop;
    public double mod = 0;
    public int modifier_glisiere = 50;
    boolean rotire = false;
    ElapsedTime rotire_timp = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        //GlisieraModule glisiera = new GlisieraModule(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_lift = hardwareMap.get(DcMotorEx.class, "motor_lift");

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_lift.setDirection(DcMotorSimple.Direction.REVERSE);


        //glisiera.init();

        waitForStart();

        if (isStopRequested()) return;

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
                            gamepad1.right_stick_y*mod,
                            gamepad1.right_stick_x*mod,
                            gamepad1.left_stick_x*mod
                    )
            );

            drive.update();


            /*
            automatizare pentru un singur buton
            f(gamepad2.x && rotire == false && rotire_timp.milliseconds() > 500) {
                rotire = true;
                rotire_timp.reset();
            }
            if(rotire == true) {
                motor_intake.setPower(1);
            }
            if(gamepad1.x && rotire == true && rotire_timp.milliseconds() > 750) {
                rotire = false;
                rotire_timp.reset();
            }
            if(rotire == false) {
                motor_intake.setPower(0);
            }

             */

            /*
            test ridicare glisiere
            if(gamepad1.a) {
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition() + modifier_glisiere);
                motorST.setTargetPosition(motorST.getCurrentPosition() + modifier_glisiere);
            }
            else if(gamepad1.b) {
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition() - modifier_glisiere);
                motorST.setTargetPosition(motorST.getCurrentPosition() - modifier_glisiere);

            }
            else {
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition());
                motorST.setTargetPosition(motorST.getCurrentPosition());
            }
            if(gamepad1.dpad_up) {
                modifier_glisiere++;
            }
            else if(gamepad1.dpad_down) {
                modifier_glisiere--;
            }

             */

            if(gamepad2.x) {
                motor_intake.setPower(0.6);
            }
            else if(gamepad2.y){
                motor_intake.setPower(-0.8);
            }
            else {
                motor_intake.setPower(0);
            }

            /*
            modul vechi de a ridica si de a cobori liftul
            if(gamepad2.a) {
                motor_lift.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(gamepad2.b) {
                motor_lift.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if(gamepad2.right_bumper) {
                motor_lift.setPower(-1);
            }
            else {
                motor_lift.setPower(0);
            }

             */

            /*
            if(gamepad1.right_bumper) {
                stop.setPosition(0);
            }
            if(gamepad1.left_bumper) {
                stop.setPosition(0);
            }

             */



            telemetry.addData("Directie", motor_lift.getDirection());

            telemetry.update();
        }

    }
}
