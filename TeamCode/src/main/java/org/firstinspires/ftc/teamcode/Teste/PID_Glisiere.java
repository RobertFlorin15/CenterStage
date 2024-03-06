package org.firstinspires.ftc.teamcode.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Config
@TeleOp (name = "Tunare PID glisiere")
public class PID_Glisiere extends LinearOpMode{
    public DcMotorEx motorST_ENC, motorDR;
    public Servo servoDR, servoST;
    int poz_min = 0;
    int poz_max = 2000;
    public static double p = 4.5, i = 0.35, d = 0.27;
    public static int target = 0;
    int modifier = 10;
    PIDController controller = new PIDController(p, i, d);
    FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorST_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorST_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorST_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller.reset();

        servoDR.setPosition(0.3);
        servoST.setPosition(0.3);

        waitForStart();

        while (opModeIsActive()) {
            update();
            //power = controller.calculate(motorDR_ENC.getCurrentPosition(), 750);
            //
            // if (gamepad1.a) {
            //     motorDR_ENC.setPower(power);
            //     motorST.setPower(power);
            // }
            //
            //  /*
            // if(gamepad1.a){
            //     if(controller.getSetPoint()  >= poz_min && controller.getSetPoint() + modifier <= poz_max){
            //         controller.setSetPoint(Range.clip(controller.getSetPoint() + modifier, poz_min, poz_max ));
            //     }
            // }
            //
            // else if(gamepad1.b){
            //     if(controller.getSetPoint() - modifier >= poz_min && controller.getSetPoint() <= poz_max){
            //         controller.setSetPoint(Range.clip(controller.getSetPoint() - modifier, poz_min, poz_max ));
            //     }
            // }
            if (gamepad1.x) {
                controller.setSetPoint(target);
            }

            else {
                controller.setSetPoint(controller.getSetPoint());
            }



            dashboard.updateConfig();
            telemetry.addData("A AJUNS?", controller.atSetPoint());
            telemetry.addData("Pozitia ST: ", motorST_ENC.getCurrentPosition());
            telemetry.addData("Pozitie DR: ", motorDR.getCurrentPosition());
            telemetry.addData("Controller pozition: ", controller.getSetPoint());
            telemetry.addData("Pozitia care trb atinsa", controller.getSetPoint());
            //telemetry.addData("Eroare Pozitie", controller.getPositionError());
            telemetry.update();
        }
    }

    public void update() {
        controller.setPID(p, i, d);
        if (!controller.atSetPoint() || motorST_ENC.getCurrentPosition() != controller.getSetPoint()) {
            double output = controller.calculate(
                    motorST_ENC.getCurrentPosition()
            );
            motorST_ENC.setVelocity(output);
            motorDR.setVelocity(output);
        }
    }
}