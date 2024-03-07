package org.firstinspires.ftc.teamcode.Regionala.Modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    PIDController controller = new PIDController(kp, ki, kd);

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
        //opritor = hardwareMap.get(Servo.class, "opritor");


        CR7.setDirection(DcMotorSimple.Direction.REVERSE);

        coborare_intake.setPosition(0.5355);

        //brat
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva");
        cuva_inchidere = hardwareMap.get(Servo.class, "cuva_inchidere");

        servoST.setPosition(0);
        servoDR.setPosition(0);
        rotire_cuva.setPosition(0.1044);
        cuva_inchidere.setPosition(0);
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
        controller.setSetPoint(250);
        servoDR.setPosition(0.72722); //0.7005
        servoST.setPosition(0.72722);
        rotire_cuva.setPosition(0.2933); //0.24944
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
        motor_intake.setPower(-0.5);
        CR7.setPower(-1);
    }

    public void trage1() {
        motor_intake.setPower(1);
        CR7.setPower(1);
        coborare_intake.setPosition(0.8);
    }

    public void trage2() {
        motor_intake.setPower(1);
        CR7.setPower(1);
    }

    public void stop() {
        motor_intake.setPower(0);
        CR7.setPower(0);
    }


}
