package org.firstinspires.ftc.teamcode.Regionala.Modules;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorModule {
    HardwareMap hardwareMap;

    public SensorModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    ColorRangeSensor sensor1, sensor2;
    boolean has_detected1, has_detected2;

    public void init() {
        sensor1 = hardwareMap.get(ColorRangeSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorRangeSensor.class, "sensor2");
    }

    public int detectare() {
        if (sensor1.getDistance(DistanceUnit.CM) < 1.5 && sensor2.getDistance(DistanceUnit.CM) < 1.5) return 1;
        else return 0;
    }

}
