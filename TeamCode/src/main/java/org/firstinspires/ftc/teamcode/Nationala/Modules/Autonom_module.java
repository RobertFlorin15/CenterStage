package org.firstinspires.ftc.teamcode.Nationala.Modules;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Autonom_module {
    HardwareMap hardwareMap;

    public Autonom_module(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Servo servoDR, servoST, rotire_cuva, cuva_inchidere, coborare_intake, opritor;
    public static double kp = 4.5, ki = 0.35, kd = 0.27;
    public DcMotorEx motorST_ENC = null;
    public DcMotorEx motorDR = null;
    public DcMotorEx motor_intake;
    public CRServo CR7;

    ElapsedTime timp_prindere = new ElapsedTime();
    PIDController controller = new PIDController(kp, ki, kd);

    ColorRangeSensor sensor1, sensor2;

    ElapsedTime timp = new ElapsedTime();

    public void init() {
        //glisiere
        motorST_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");

        motorST_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorST_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDR.setDirection(DcMotorEx.Direction.REVERSE);

        controller.reset();

        //intake
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        CR7 = hardwareMap.get(CRServo.class, "CR7");
        coborare_intake = hardwareMap.get(Servo.class, "coborare_intake");

        sensor1 = hardwareMap.get(ColorRangeSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorRangeSensor.class, "sensor2");
        //opritor = hardwareMap.get(Servo.class, "opritor");


        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        coborare_intake.setPosition(0);

        //brat
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva");
        cuva_inchidere = hardwareMap.get(Servo.class, "cuva_inchidere");

        opritor = hardwareMap.get(Servo.class, "opritoare_sus");

        servoST.setPosition(0);
        servoDR.setPosition(0);
        rotire_cuva.setPosition(0.1044);
        //opritor.setPosition(0.234);
        cuva_inchidere.setPosition(0);
        timp_prindere.reset();
    }
}