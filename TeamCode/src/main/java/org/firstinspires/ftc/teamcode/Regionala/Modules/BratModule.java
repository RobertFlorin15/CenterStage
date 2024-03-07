package org.firstinspires.ftc.teamcode.Regionala.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BratModule {
    HardwareMap hardwareMap;

    public BratModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Servo servoDR, servoST, rotire_cuva, cuva_inchidere;

    public void init() {
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva");
        cuva_inchidere = hardwareMap.get(Servo.class, "cuva_inchidere");

        servoST.setPosition(0);
        servoDR.setPosition(0);
        rotire_cuva.setPosition(0.1044);
        cuva_inchidere.setPosition(0);

    }

    public void goDown() {
        servoST.setPosition(0.0244);
        servoDR.setPosition(0.0244);
        rotire_cuva.setPosition(0.1044);

    }

    public void goUp() {
        servoST.setPosition(0.575);
        servoDR.setPosition(0.575);
        rotire_cuva.setPosition(0.1044);
    }
    public void open() {
        cuva_inchidere.setPosition(0.432);
    }
    public void close() {cuva_inchidere.setPosition(0);}

    public void autonom() {
        servoDR.setPosition(0.64555);
        servoST.setPosition(0.64555);
        rotire_cuva.setPosition(0.19722);

    }

}
