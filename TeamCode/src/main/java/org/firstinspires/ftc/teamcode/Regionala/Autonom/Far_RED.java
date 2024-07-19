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

@Autonomous (name = "Roșu departe")
public class Far_RED extends LinearOpMode {
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

        Pose2d startPose = new Pose2d (-36 , -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-49.5, -45))
                .lineTo(new Vector2d(-36, -63))
                .turn(Math.toRadians(-91))
                .waitSeconds(8)
                .lineTo(new Vector2d(10, -60))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(0.4)
                .splineToConstantHeading(new Vector2d(49.3, -29.5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(1.2)
                .lineTo(new Vector2d(42.5, -29.5))
                .strafeTo(new Vector2d(44, -20))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)
                //.lineTo(new Vector2d(75, -64))

                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -37))
                .lineTo(new Vector2d(-36, -63))
                .turn(Math.toRadians(-91))
                .waitSeconds(8)
                .lineTo(new Vector2d(10, -60))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(0.4)
                .splineToConstantHeading(new Vector2d(49.3, -37.5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(1.2)
                .lineTo(new Vector2d(42.5, -40))
                .strafeTo(new Vector2d(44, -20))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)



                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -45))
                .turn(Math.toRadians(-91))
                .lineTo(new Vector2d(-35, -45))
                .lineTo(new Vector2d(-36, -45))
                .strafeTo(new Vector2d(-36, -63))
                .waitSeconds(4)
                .lineTo(new Vector2d(10, -60))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(0.4)
                .splineToConstantHeading(new Vector2d(49.3, -44), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(1.2)
                .lineTo(new Vector2d(42.5, -44))
                .strafeTo(new Vector2d(44, -20))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)

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
