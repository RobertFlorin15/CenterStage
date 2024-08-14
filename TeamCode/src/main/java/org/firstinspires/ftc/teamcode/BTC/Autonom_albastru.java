package org.firstinspires.ftc.teamcode.BTC;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Detection.blue.DetectionClass_blue;
import org.firstinspires.ftc.teamcode.Detection.blue.TeamProp_blue;
import org.firstinspires.ftc.teamcode.Nationala.Modules.Autonom_module;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous (name = "albastru...")
public class Autonom_albastru extends LinearOpMode {
    public ElapsedTime time = new ElapsedTime();
    public class Glisiera {
        private DcMotorEx motorDR, motorST_ENC;
        double kp = 4.5, ki = 0.35 , kd = 0.27;
        PIDController controller = new PIDController(kp, ki, kd);

        public Glisiera(HardwareMap hardwareMap) {
            motorST_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
            motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");

            motorST_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motorST_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorDR.setDirection(DcMotorEx.Direction.REVERSE);


        }

        public void update() {
            controller.setPID(kp, ki, kd);
            if (!controller.atSetPoint() || motorST_ENC.getCurrentPosition() != controller.getSetPoint()) {
                double output = controller.calculate(
                        motorST_ENC.getCurrentPosition()
                );
                motorST_ENC.setVelocity(output);
                motorDR.setVelocity(output);
            }

        }

        public class GoUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized) {
                    controller.setSetPoint(450);
                    initialized = true;
                }

                if(!controller.atSetPoint()) {
                    update();
                    return true;
                }
                return false;

                /*
                else {
                    double output;
                    output = controller.calculate(motorST_ENC.getCurrentPosition());
                    motorDR.setVelocity(output);
                    motorST_ENC.setVelocity(output);
                    return false;
                }
                 */

            }
        }

        public Action GoUp() {
            return new GoUp();
        }

        public class GoDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    controller.setSetPoint(0);
                    initialized = true;
                }

                if (!controller.atSetPoint()) {
                    update();
                    return true;
                }
                return false;
            }
        }

        public Action GoDown() {
            return new GoDown();
        }

    }


    public class Brat {
        private Servo servoDR, servoST, rotire_cuva, cuva_inchidere;

        public Brat(HardwareMap hardwareMap) {
            servoDR = hardwareMap.get(Servo.class, "servoDR");
            servoST = hardwareMap.get(Servo.class, "servoST");
            rotire_cuva = hardwareMap.get(Servo.class, "rotire_cuva");
            cuva_inchidere = hardwareMap.get(Servo.class, "cuva_inchidere");
        }

        public class GoUp implements Action {
            
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoDR.setPosition(0.609);
                servoST.setPosition(0.609);
                rotire_cuva.setPosition(0.2605);
                return false;
            }
        }

        public Action GoUp() {
            return new GoUp();
        }

        public class GoDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoDR.setPosition(0.06);
                servoST.setPosition(0.06);
                rotire_cuva.setPosition(0.10388);
                return false;
            }
        }

        public Action GoDown() {
            return new GoDown();
        }

        public class OpenCuva implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                cuva_inchidere.setPosition(0.432);
                return false;
            }
        }

        public Action OpenCuva() {
            return new OpenCuva();
        }

        public class CloseCuva implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                cuva_inchidere.setPosition(0);
                return false;
            }
        }

        public Action CloseCuva() {
            return new CloseCuva();
        }
    }


    public class Intake {
        private DcMotorEx motor_intake;
        private CRServo CR7;
        private Servo coborare_intake;
        //ElapsedTime timp;

        public Intake(HardwareMap hardwareMap) {
            motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
            CR7 = hardwareMap.get(CRServo.class, "CR7");
            coborare_intake = hardwareMap.get(Servo.class, "coborare_intake");

            CR7.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class Trage implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                if(time.seconds() < 1) {
                    motor_intake.setPower(1);
                    CR7.setPower(1);
                    return true;
                }
                else {
                    motor_intake.setPower(0);
                    CR7.setPower(0);
                    return false;
                }
            }
        }

        public Action Trage() {
            return new Trage();
        }

        public class Scuipa implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                if(time.seconds() < 1) {
                    motor_intake.setPower(-0.35);
                    CR7.setPower(-1);
                    return true;
                }
                else {
                    motor_intake.setPower(0);
                    CR7.setPower(0);
                    return false;
                }
            }
        }
        public Action Scuipa() {
            return new Scuipa();
        }

        public class STOPIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                motor_intake.setPower(0);
                CR7.setPower(0);
                return false;
            }
        }

        public Action STOPIntake() {
            return new STOPIntake();
        }

        public class poz1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                coborare_intake.setPosition(0.1905);
                return false;
            }
        }

        public Action poz1() {
            return new poz1();
        }

        public class poz2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                coborare_intake.setPosition(0.37);
                return false;
            }
        }

        public Action poz2() {
            return new poz2();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, 64, Math.toRadians(-90)));
        Autonom_module module = new Autonom_module(hardwareMap);
        DetectionClass_blue detectare = new DetectionClass_blue(hardwareMap);
        TeamProp_blue.Location TeamProp_location = TeamProp_blue.Location.LEFT;


        Glisiera glisiera = new Glisiera(hardwareMap);
        Brat brat = new Brat(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        detectare.init();
        module.init();
        time.reset();


        while (!opModeIsActive()) {
            TeamProp_location = detectare.getLocation();
            telemetry.addData("Location: ", TeamProp_location);
            telemetry.update();
        }

        Action Trajectory1R;
        Action Trajectory1C;
        Action Trajectory1L;

        Action Trajectory2;

        Action Trajectory3R;
        Action Trajectory3C;
        Action Trajectory3L;

        Action Trajectory4;
        Action Trajectory5;
        Action Trajectory6;

        Trajectory1R = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(60, 32), 0)
                .build();

        Trajectory1C = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(60, 36), 0)
                .build();

        Trajectory1L = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(60, 40), 0)
                .build();


        Trajectory2 = drive.actionBuilder(drive.pose)
                .afterTime(0.5,brat.OpenCuva())
                .build();


        Trajectory3R = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(40, 32), 0)
                .build();

        Trajectory3C = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(28, 24), 0)
                .build();

        Trajectory3L = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(16, 32), 0)
                .build();


        Trajectory4 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, 12), 0)
                .splineTo(new Vector2d(-48,20), 0)
                .build();

        Trajectory5 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(-36, 12), 0)
                .splineTo(new Vector2d(12,12), 0) //sau lineToX
                .splineTo(new Vector2d(60, 36), 0)
                .build();

        Trajectory6 = drive.actionBuilder(drive.pose)
                .lineToY(65)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        if(TeamProp_location == TeamProp_blue.Location.RIGHT) {
            Actions.runBlocking(
                    new SequentialAction(
                        Trajectory1R,
                        new ParallelAction(
                            glisiera.GoUp(),
                            brat.GoUp()
                        ),
                        Trajectory2,
                        new ParallelAction(
                            glisiera.GoDown(),
                            brat.GoDown(),
                            brat.OpenCuva(),
                            Trajectory3R
                        ),
                        intake.Scuipa(),
                        Trajectory4,
                        new ParallelAction(  //la stack
                             intake.poz1(),
                             intake.Trage()
                        ),
                        Trajectory5,
                        new ParallelAction(
                             glisiera.GoUp(),
                             brat.GoUp()
                        ),
                        Trajectory2,
                        new ParallelAction(
                              Trajectory6,
                              brat.GoDown(),
                              glisiera.GoDown(),
                              brat.CloseCuva()
                        )
                    )
            );
        }

        else if(TeamProp_location == TeamProp_blue.Location.CENTER) {
            Actions.runBlocking(
                    new SequentialAction(
                            Trajectory1C,
                            new ParallelAction(
                                    glisiera.GoUp(),
                                    brat.GoUp()
                            ),
                            Trajectory2,
                            new ParallelAction(
                                    glisiera.GoDown(),
                                    brat.GoDown(),
                                    brat.OpenCuva(),
                                    Trajectory3C
                            ),
                            intake.Scuipa(),
                            Trajectory4,
                            new ParallelAction(  //la stack
                                    intake.poz1(),
                                    intake.Trage()
                            ),
                            Trajectory5,
                            new ParallelAction(
                                    glisiera.GoUp(),
                                    brat.GoUp()
                            ),
                            Trajectory2,
                            new ParallelAction(
                                    Trajectory6,
                                    brat.GoDown(),
                                    glisiera.GoDown(),
                                    brat.CloseCuva()
                            )
                    )
            );
        }

        else {
            Actions.runBlocking(
                    new SequentialAction(
                            Trajectory1L,
                            new ParallelAction(
                                    glisiera.GoUp(),
                                    brat.GoUp()
                            ),
                            Trajectory2,
                            new ParallelAction(
                                    glisiera.GoDown(),
                                    brat.GoDown(),
                                    brat.OpenCuva(),
                                    Trajectory3L
                            ),
                            intake.Scuipa(),
                            Trajectory4,
                            new ParallelAction(  //la stack
                                    intake.poz1(),
                                    intake.Trage()
                            ),
                            Trajectory5,
                            new ParallelAction(
                                    glisiera.GoUp(),
                                    brat.GoUp()
                            ),
                            Trajectory2,
                            new ParallelAction(
                                    Trajectory6,
                                    brat.GoDown(),
                                    glisiera.GoDown(),
                                    brat.CloseCuva()
                            )
                    )
            );
        }
    }
}