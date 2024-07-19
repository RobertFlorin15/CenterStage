package org.firstinspires.ftc.teamcode.Nationala.Modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonomModule {
    HardwareMap hardwareMap;
    public AutonomModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public Servo servoDR, servoST, rotire_cuva, cuva_inchidere, coborare_intake, opritor;
    public static double kp = 4.5, ki = 0.35 , kd = 0.27;
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
        opritor.setPosition(0.234);
        cuva_inchidere.setPosition(0);
        timp_prindere.reset();
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

    public void goUp1() {
        controller.setSetPoint(200);
        servoDR.setPosition(0.575); //0.7005
        servoST.setPosition(0.575);
        rotire_cuva.setPosition(0.2605); //0.24944
        coborare_intake.setPosition(0.535); //0.535
    }

    public void goDown() {
        controller.setSetPoint(0);
        servoST.setPosition(0.0244);
        servoDR.setPosition(0.0244);
        rotire_cuva.setPosition(0.1044);
        cuva_inchidere.setPosition(0);
    }

    public void open() {
        cuva_inchidere.setPosition(0.432);
    }

    public void scuipa() {
        motor_intake.setPower(-0.35);
        CR7.setPower(-1);
    }

    public void trage() {
        motor_intake.setPower(1);
        CR7.setPower(1);
    }

    public void stop() {
        motor_intake.setPower(0);
        CR7.setPower(0);
    }

    public void coborare_intake1() {
        coborare_intake.setPosition(0.2);
    }

    public void glisiera_departe() {
        controller.setSetPoint(200);
    }

    public void ridica_pentrustack() {
        coborare_intake.setPosition(0.1905);
    }
    public void trage_auto1(){
        coborare_intake.setPosition(0.1905);
        motor_intake.setPower(1);
        CR7.setPower(1);
    }

    public void trage_auto2(){
        coborare_intake.setPosition(0.37);
        motor_intake.setPower(1);
        CR7.setPower(1);
    }

    public void scuipa_perie() {
        motor_intake.setPower(-0.5);
    }

    //0.2166


}
