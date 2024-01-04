package org.firstinspires.ftc.teamcode.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp (name = "PID_GalatiPOCPOCMIAUBAU", group = "Teste")
public class GlisieraPID_test extends LinearOpMode {
    FtcDashboard dashboard;
    public static double kp = 0.0, kd = 0.0, ki = 0.0;
    public DcMotorEx motorST, motorDR_ENC;
    int modifier = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorST = hardwareMap.get(DcMotorEx.class, "motorST");
        motorDR_ENC = hardwareMap.get(DcMotorEx.class, "motorDR_ENC");

        motorST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorST.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDR_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDR_ENC.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDController controller = new PIDController(0.0, 0.0, 0.0);

        controller.reset();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_up) {
                modifier++;
            }

            else if(gamepad1.dpad_down) {
                modifier--;
            }

            if(gamepad1.a) {
                //if(controller.getSetPoint() >=)
            }
        }

    }
}
