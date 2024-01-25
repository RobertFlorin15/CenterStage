package org.firstinspires.ftc.teamcode.Meeturi.Module;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BratModule {
    HardwareMap hardwareMap;
    public BratModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    Servo servoDR, servoST, servo_extindere = null;
    double modifier = 0.0001;

    double change_modifier = 0.00000005;

    public void init() {
        servoDR = hardwareMap.get(Servo.class, "servoDR");
        servoST = hardwareMap.get(Servo.class, "servoST");
        servo_extindere = hardwareMap.get(Servo.class, "servo_extindere");
        servoDR.setPosition(0.0822222);
        servoST.setPosition(0.0822222);
        servo_extindere.setPosition(0.37777);
    }

    //ridicarea bratului
    public void goDown() {
        servoDR.setPosition(0.0822222);
        servoST.setPosition(0.0822222);
    }


    //extinderea bratului

    public void retras() {
        servo_extindere.setPosition(0);
    }

    public void interior() {
        servo_extindere.setPosition(0.37777);
    }

    public void exterior() {
        servo_extindere.setPosition(0.635);
    }
}





