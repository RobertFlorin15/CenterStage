package org.firstinspires.ftc.teamcode.Regionala.Autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Detection.red.DetectionClass_red;
import org.firstinspires.ftc.teamcode.Detection.red.TeamProp_red;
import org.firstinspires.ftc.teamcode.Nationala.Modules.AutonomModule;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Roșu aproape")
public class Close_RED extends LinearOpMode {
    Trajectory traj1, traj2, traj3;
    ColorRangeSensor sensor1, sensor2;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutonomModule module = new AutonomModule(hardwareMap);

        DetectionClass_red detectare = new DetectionClass_red(hardwareMap);
        TeamProp_red.Location TeamProp_location = TeamProp_red.Location.LEFT;

        detectare.init();
        module.init();

        while (!opModeIsActive()) {
            TeamProp_location = detectare.getLocation();
            telemetry.addData("Location: ", TeamProp_location);
            telemetry.update();
        }

        waitForStart();

        Pose2d startPose = new Pose2d (36 , -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(73.7, -29), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .lineTo(new Vector2d(35.5, -34))
                .UNSTABLE_addTemporalMarkerOffset(0, module::scuipa)
                .waitSeconds(2)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(0.5)
                //.strafeTo(new Vector2d(34, -10))
                //.lineTo(new Vector2d(-33, -16)) -stack
                /*.UNSTABLE_addTemporalMarkerOffset(0, module::trage1)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(34, -10))
                .addTemporalMarker(11, module::opritor_deschidere)
                .waitSeconds(0.1)
                .addTemporalMarker(11.1, module::trage1)
                .lineTo(new Vector2d(63.2, -44))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goUp1)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0.5, module::goDown)
                 */
                .lineTo(new Vector2d(75, -64))

                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(73.9, -37.5), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .lineTo(new Vector2d(48.4, -24.3))
                .UNSTABLE_addTemporalMarkerOffset(0, module::scuipa)
                .waitSeconds(0.8)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(0.5)
                //.strafeTo(new Vector2d(45, -10))
                .lineTo(new Vector2d(75, -64))



                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(73.9, -45.27), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .lineTo(new Vector2d(57.1, -30))
                .UNSTABLE_addTemporalMarkerOffset(0, module::scuipa)
                .waitSeconds(1.25)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(0.5)
                /*.strafeTo(new Vector2d(58, -10))
                .lineTo(new Vector2d(34, -10))

                 */

                .lineTo(new Vector2d(75, -64))

                .build();

        if (TeamProp_location == TeamProp_red.Location.LEFT) {
            drive.followTrajectorySequenceAsync(traj1);
        }

        else if (TeamProp_location == TeamProp_red.Location.CENTER) {
            drive.followTrajectorySequenceAsync(traj2);
        }

        else {
            drive.followTrajectorySequenceAsync(traj3);
        }



        while (opModeIsActive()) {
            drive.update();
            module.update();
        }




    }

}
