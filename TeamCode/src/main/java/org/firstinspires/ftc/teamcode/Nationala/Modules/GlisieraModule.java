package org.firstinspires.ftc.teamcode.Nationala.Modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class GlisieraModule {
    HardwareMap hardwareMap;

    public GlisieraModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public static double kp = 4.5, ki = 0.35 , kd = 0.27;
    public DcMotorEx motorST_ENC = null;
    public DcMotorEx motorDR = null;
    PIDController controller = new PIDController(kp, ki, kd);

    int modifier = 1;

    int poz_max = 2000;
    int poz_min = 0;


    public void init() {

        motorST_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");

        motorST_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorST_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDR.setDirection(DcMotorEx.Direction.REVERSE);

        controller.reset();

    }

    public void update() {
        controller.setPID(kp, ki, kd);
        if (!controller.atSetPoint() || motorST_ENC.getCurrentPosition() != controller.getSetPoint()) {
            double output = controller.calculate(
                    motorST_ENC.getCurrentPosition()
            );
            motorST_ENC.setVelocity(output);
            motorDR.setVelocity(output);
        }
    }

    public void goUp(){
        controller.setSetPoint(2000);
    }
    public void goUp_Ciprica(){
        controller.setSetPoint(1800);
    }

    public void goDown(){
        controller.setSetPoint(0);
    }

    public void goMid(){
        controller.setSetPoint(850);
    }
    public void goLow_Ciprica() {
        controller.setSetPoint(250);
    }
    public void goLow() {
        controller.setSetPoint(450);
    }
    public void goDown_autonom() {
        controller.setSetPoint(1300);
    }

    public void goNicu() {
        controller.setSetPoint(150);
    }


}

