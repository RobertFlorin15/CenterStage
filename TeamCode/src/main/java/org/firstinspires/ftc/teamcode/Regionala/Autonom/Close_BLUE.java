package org.firstinspires.ftc.teamcode.Regionala.Autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Detection.blue.DetectionClass_blue;
import org.firstinspires.ftc.teamcode.Detection.blue.TeamProp_blue;
import org.firstinspires.ftc.teamcode.Nationala.Modules.AutonomModule;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "BLUE aproape")
public class Close_BLUE extends LinearOpMode {
    Trajectory traj1, traj2, traj3;
    ColorRangeSensor sensor1, sensor2;
    boolean functie = false;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutonomModule module = new AutonomModule(hardwareMap);

        DetectionClass_blue detectare = new DetectionClass_blue(hardwareMap);
        TeamProp_blue.Location TeamProp_location = TeamProp_blue.Location.LEFT;

        detectare.init();
        module.init();

        while (!opModeIsActive()) {
            TeamProp_location = detectare.getLocation();
            telemetry.addData("Location: ", TeamProp_location);
            telemetry.update();
        }

        waitForStart();

        Pose2d startPose = new Pose2d (36 ,64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(74.7, 29.7), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.12)
                .lineTo(new Vector2d(34.9, 35))
                .UNSTABLE_addTemporalMarkerOffset(0.1, module::scuipa)
                .waitSeconds(1.8)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(8)
                //.UNSTABLE_addTemporalMarkerOffset(0.2, module::trage_auto)

                //.strafeTo(new Vector2d(58, 10))
                //.lineTo(new Vector2d(-34, 10))
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
                .splineToConstantHeading(new Vector2d(82, 64), 0)



                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(75, 35.3), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .lineTo(new Vector2d(47.4, 26))
                .UNSTABLE_addTemporalMarkerOffset(0, module::scuipa)
                .waitSeconds(1)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(0.5)
                //.strafeTo(new Vector2d(45, -10))
                .splineToConstantHeading(new Vector2d(82, 64), 0)

                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(74.7, 45.27), 0)
                .addTemporalMarker(0.5, module::goUp1)
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0, module::open)
                .waitSeconds(0.07)
                .lineTo(new Vector2d(57, 30))
                .UNSTABLE_addTemporalMarkerOffset(0, module::scuipa)
                .waitSeconds(1.25)
                .addTemporalMarker(4, module::goDown)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::stop)
                .waitSeconds(0.5)
                /*.strafeTo(new Vector2d(58, -10))
                .lineTo(new Vector2d(34, -10))

                 */

                .splineToConstantHeading(new Vector2d(82, 64), 0)

                .build();

        if (TeamProp_location == TeamProp_blue.Location.RIGHT) {
            drive.followTrajectorySequenceAsync(traj1);
        }

        else if (TeamProp_location == TeamProp_blue.Location.CENTER) {
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