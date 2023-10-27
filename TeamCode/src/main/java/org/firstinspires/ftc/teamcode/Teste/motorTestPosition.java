package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Test_motorPozitie", group = "Test")
public class motorTestPosition extends LinearOpMode {

    public DcMotorEx motor_testat = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_testat = hardwareMap.get(DcMotorEx.class, "motor_brat");
        motor_testat.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.right_bumper) {
                motor_testat.setPower(0.5);
                telemetry.addData("Pozitie motor", motor_testat.getCurrentPosition());
            }
            else{
                motor_testat.setPower(0);
            }
            telemetry.update();
        }
    }


}
