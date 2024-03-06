package org.firstinspires.ftc.teamcode.Regionala;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Detection.red.DetectionClass_red;
import org.firstinspires.ftc.teamcode.Detection.red.TeamProp_red;
import org.firstinspires.ftc.teamcode.Regionala.Modules.BratModule;
import org.firstinspires.ftc.teamcode.Regionala.Modules.GlisieraModule;
import org.firstinspires.ftc.teamcode.Regionala.Modules.IntakeModule;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "PRIMUL AUTONOM PE ODOMETRII")
public class Autonom_aproapeBLUE extends LinearOpMode {
    Trajectory traj1;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BratModule brat = new BratModule(hardwareMap);
        GlisieraModule glisiera = new GlisieraModule(hardwareMap);
        IntakeModule intake = new IntakeModule(hardwareMap);

        DetectionClass_red detectare = new DetectionClass_red(hardwareMap);
        TeamProp_red.Location TeamProp_location = TeamProp_red.Location.LEFT;

        detectare.init();
        brat.init();
        glisiera.init();
        intake.init();

        while (!opModeIsActive()) {
            TeamProp_location = detectare.getLocation();
            telemetry.addData("Location: ", TeamProp_location);
            telemetry.update();
        }

        waitForStart();

        Pose2d startPose = new Pose2d (36 , -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36 ,-37 ))
                .lineTo(new Vector2d(36 , -40))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(60,-37))
                .UNSTABLE_addTemporalMarkerOffset(0.2, glisiera::goMid)

                //.UNSTABLE_addTemporalMarkerOffset(1, brat::goUp)
                .build();

        drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive()) {
            drive.update();
            glisiera.update();
        }




    }

}
