package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp (name = "Testam teleOP-ul emoji luna")
public class Test_TeleOP extends LinearOpMode {

    public Servo servoDR, servoST, rotire_cuva, servo_intake;
    public DcMotorEx motor_intake;
    public CRServo CR7;
    public DcMotorEx motorDR, motorST;
    //pozitie coborare perie: init:0.0 jos:0.5355

    @Override
    public void runOpMode() throws InterruptedException {
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva");
        servo_intake = hardwareMap.get(Servo.class, "servo_intake");

        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");

        CR7 = hardwareMap.get(CRServo.class, "CR7");

        CR7.setDirection(DcMotorSimple.Direction.REVERSE);
        motorST.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDR.setDirection(DcMotorSimple.Direction.FORWARD);


        servoDR.setPosition(0.06111);
        servoST.setPosition(0.06111);
        rotire_cuva.setPosition(0.10388);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                servoDR.setPosition(0.06111);
                servoST.setPosition(0.06111);
                rotire_cuva.setPosition(0.10388);
            }

            if (gamepad1.dpad_left) {
                servoDR.setPosition(0.4);
                servoST.setPosition(0.4);
                rotire_cuva.setPosition(0.1044);
            }

            if (gamepad1.right_bumper) {
                motorDR.setPower(0.7);
                motorST.setPower(0.7);
            }

            else if (gamepad1.left_bumper){
                motorDR.setPower(-1);
                motorST.setPower(-1);
            }

            else {
                motorDR.setPower(0);
                motorST.setPower(0);
            }

            if (gamepad1.a) {
                motor_intake.setPower(1);
                CR7.setPower(1);
            }

            else {
                motor_intake.setPower(0);
                CR7.setPower(0);
            }
        }
    }
}
