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

@Autonomous (name = "Albastru departe")
public class Far_BLUE extends LinearOpMode {
    Trajectory traj1, traj2, traj3;
    ColorRangeSensor sensor1, sensor2;
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

        Pose2d startPose = new Pose2d (-36 , 64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)

                .lineTo(new Vector2d(-36, 40))
                .turn(Math.toRadians(91))
                .lineTo(new Vector2d(-35, 40))
                .lineTo(new Vector2d(-36, 40))
                .strafeTo(new Vector2d(-36, 64))
                .lineTo(new Vector2d(10, 65))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(3.4)
                .splineToConstantHeading(new Vector2d(47, 38), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(1.2)
                .strafeTo(new Vector2d(44, 64))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)
                //.lineTo(new Vector2d(75, -64))



                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 37))
                .lineTo(new Vector2d(-36, 64))
                .turn(Math.toRadians(91))
                .splineToConstantHeading(new Vector2d(10, 64), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(5.4)
                .splineToConstantHeading(new Vector2d(47, 25), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(0.8)
                .lineTo(new Vector2d(44, 25))
                .strafeTo(new Vector2d(44, 64))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)



                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-50, 43.7))
                .lineTo(new Vector2d(-36, 64))
                .turn(Math.toRadians(91))
                .splineToConstantHeading(new Vector2d(10, 64), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, module::glisiera_departe)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, module::goUp1)
                .waitSeconds(5.4)
                .splineToConstantHeading(new Vector2d(47, 18), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.7, module::open)
                .waitSeconds(0.8)
                .lineTo(new Vector2d(44, 20.7))
                .strafeTo(new Vector2d(44, 63))
                .UNSTABLE_addTemporalMarkerOffset(0.2, module::goDown)
                .waitSeconds(0.5)

                .build();

        if (TeamProp_location == TeamProp_blue.Location.LEFT) {
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
