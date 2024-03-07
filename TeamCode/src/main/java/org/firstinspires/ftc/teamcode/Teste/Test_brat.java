package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Regionala.Modules.GlisieraModule;

@TeleOp (name = "Test BRAT FORTA AXON te rog sa nu bubuie iar robotul")
public class Test_brat extends LinearOpMode {

    public Servo servoDR, servoST, rotire_cuva, servo_inchidere;
    double modifier = 0.0001;
    @Override
    public void runOpMode() throws InterruptedException {
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva"); //rotire cuva
        //servo_inchidere = hardwareMap.get(Servo.class, "servo_inchidere");
        //pozitie orientativa petru tablita brat 0.6027

        servoST.setPosition(0);
        servoDR.setPosition(0);
        rotire_cuva.setPosition(0.1044); //prindere pixeli
        //servo_inchidere.setPosition(0);

        GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);

        glisieraModule.init();

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                servoDR.setPosition(0);
                servoST.setPosition(0);
                rotire_cuva.setPosition(0.1044);
            }

            if (gamepad1.a) {
                servoST.setPosition(servoST.getPosition() + modifier);
                servoDR.setPosition(servoDR.getPosition() + modifier);
                //rotire_cuva.setPosition(rotire_cuva.getPosition() + modifier);
            }

            if (gamepad1.b) {
                servoST.setPosition(servoST.getPosition() - modifier);
                servoDR.setPosition(servoDR.getPosition() - modifier);
                //rotire_cuva.setPosition(rotire_cuva.getPosition() - modifier);
            }

            if (gamepad1.x) {
                rotire_cuva.setPosition(rotire_cuva.getPosition() + modifier);
            }

            if (gamepad1.y) {
                rotire_cuva.setPosition(rotire_cuva.getPosition() - modifier);
            }

            if (gamepad1.right_bumper) {
                servoDR.setPosition(0.3);
                servoST.setPosition(0.3);
            }

            if (gamepad1.dpad_up) {
                glisieraModule.goLow();
            }

            glisieraModule.update();


            telemetry.addData("Pozitie DR: ", servoDR.getPosition());
            telemetry.addData("Pozitie ST: ", servoST.getPosition());
            telemetry.addData("Cuva:", rotire_cuva.getPosition());
            //telemetry.addData("Inchidere: ", servo_inchidere.getPosition());
            telemetry.update();

        }
    }
}
