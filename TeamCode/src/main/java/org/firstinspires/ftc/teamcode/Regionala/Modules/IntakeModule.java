package org.firstinspires.ftc.teamcode.Regionala.Modules;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeModule {
    HardwareMap hardwareMap;

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DcMotorEx motor_intake;
    public CRServo CR7;

    public Servo coborare_intake;

    public void init() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        coborare_intake = hardwareMap.get(Servo.class, "coborare_intake");

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        coborare_intake.setPosition(0.5355);

    }

    public void mananca_pixeli() {
        motor_intake.setPower(1);
        CR7.setPower(1);
    }
    public void nu_mananca_pixeli(){
        motor_intake.setPower(0);
        CR7.setPower(0);
    }
    public void da_la_rate_pixeli() {
        motor_intake.setPower(-1);
        CR7.setPower(-1);
    }
}
