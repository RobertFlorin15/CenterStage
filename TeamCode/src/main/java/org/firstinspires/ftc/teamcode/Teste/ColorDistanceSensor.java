package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp (name = "Test senzor de distanta care e de fapt de culoare, el nebun")
public class ColorDistanceSensor extends LinearOpMode {
    ColorRangeSensor sensor1, sensor2;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor1 = hardwareMap.get(ColorRangeSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorRangeSensor.class, "sensor2");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance_1: ", sensor1.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance_2: ", sensor2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}