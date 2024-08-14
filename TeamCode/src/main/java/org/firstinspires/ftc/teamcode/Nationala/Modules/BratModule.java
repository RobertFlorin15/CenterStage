package org.firstinspires.ftc.teamcode.Nationala.Modules;

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

        servoST.setPosition(0.0485); //0.0433
        servoDR.setPosition(0.0485);
        rotire_cuva.setPosition(0.10388); //0.1044
        cuva_inchidere.setPosition(0);

    }

    public void goDown() {
        servoST.setPosition(0.06); //0.0444
        servoDR.setPosition(0.06);
        rotire_cuva.setPosition(0.10388);//0.1044

    }

    public void goUp() {
        servoST.setPosition(0.609); //0.575
        servoDR.setPosition(0.609);
        rotire_cuva.setPosition(0.2605);
    }
    public void open() {
        cuva_inchidere.setPosition(0.432);
    }
    public void close() {cuva_inchidere.setPosition(0);}

    public void autonom() {
        servoDR.setPosition(0.6795); //0.64555
        servoST.setPosition(0.6795);
        rotire_cuva.setPosition(0.19722);

    }
}
