package org.firstinspires.ftc.teamcode.Detection.blue;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "Blue vision test")
public class TestBlue {
    TeamProp_blue detectionBlue = new TeamProp_blue();
    OpenCvWebcam webcam;

    //problema vine de la HardwareMap, daca nu functioneaza asta trebuie sa cream o variabila hardware map + o functie ce aloca hardwareMap-ului o piesa
   /* @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("webcamID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, webcamID);
    }

    */

}
