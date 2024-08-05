package org.firstinspires.ftc.teamcode.Vechi.Regionala;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.imgproc.CLAHE;

@TeleOp (name = "Distance")
public class Distance extends LinearOpMode {
    DistanceSensor distanceSensor;
    ColorRangeSensor colorSensor;
    boolean has_detected1, has_detected2;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            if (distanceSensor.getDistance(DistanceUnit.CM) < 3) {
                has_detected1 = true;
            }

            /*
            if () {
                has_detected2 = true;
            }
             */

            telemetry.addData("CIPrica: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("ROBert: ", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}
