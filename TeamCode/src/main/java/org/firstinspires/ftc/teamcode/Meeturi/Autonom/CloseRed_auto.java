package org.firstinspires.ftc.teamcode.Meeturi.Autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Detection.red.DetectionClass_red;
import org.firstinspires.ftc.teamcode.Detection.red.TeamProp_red;

@Autonomous(name = "Rosu aproape")
public class CloseRed_auto extends LinearOpMode {
    DcMotorEx rightFront, rightBack, leftFront, leftBack, motor_intake;


    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.3;
    public Servo avion;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime rotire = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        avion = hardwareMap.get(Servo.class, "avion");
        DetectionClass_red detectare = new DetectionClass_red(hardwareMap);
        TeamProp_red.Location TeamProp_location = TeamProp_red.Location.LEFT;

        //motoarele din stanga sunt in onglida, asa ca primesc REVERSE pentru a se roti in directia FORWARD
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_intake.setDirection(DcMotorSimple.Direction.FORWARD);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Starting at", "%7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition());
        telemetry.update();


        detectare.init();

        avion.setPosition(0.64444);


        while (!opModeIsActive()) {
            TeamProp_location = detectare.getLocation();
            telemetry.addData("Location", TeamProp_location);
            telemetry.update();
        }

        waitForStart();


            if (TeamProp_location == TeamProp_red.Location.LEFT) {
                encoderDrive(DRIVE_SPEED, 27.5, 27.5, 4);
                encoderDrive(DRIVE_SPEED, -23, 23, 4);
                rotire.reset();
                while(rotire.seconds() <= 2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                encoderDrive(DRIVE_SPEED, -3.7, -3.7,4);
                encoderDrive(DRIVE_SPEED, 45, -45, 4);
                encoderDrive(0.7, 20, 20,4);
                encoderDrive(DRIVE_SPEED,9.7,9.7,2);
                rotire.reset();
                while(rotire.seconds()<=2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                rotire.reset();
            }

            else if (TeamProp_location == TeamProp_red.Location.RIGHT) {
                encoderDrive(DRIVE_SPEED, 29, 29, 4);
                encoderDrive(DRIVE_SPEED, 23, -23, 4);
                rotire.reset();
                while(rotire.seconds() <= 2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                encoderDrive(DRIVE_SPEED, -3.7, -3.7, 4);
                encoderDrive(DRIVE_SPEED, 20, -20, 2);
                encoderDrive(DRIVE_SPEED, 20, 20, 2);
                encoderDrive(DRIVE_SPEED, -20, 20, 4);
                encoderDrive(0.7, 20, 20, 4);
                encoderDrive(DRIVE_SPEED, 15, 15, 4);
                rotire.reset();
                while(rotire.seconds() <= 2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                rotire.reset();
            }

            else {
                encoderDrive(DRIVE_SPEED, 26 ,26, 4);
                rotire.reset();
                while(rotire.seconds() <= 2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                encoderDrive(DRIVE_SPEED, -4, -4, 2);
                encoderDrive(DRIVE_SPEED, 23, -23, 2);
                encoderDrive(0.7, 20, 20, 4);
                encoderDrive(DRIVE_SPEED, 11, 11, 4);
                rotire.reset();
                while(rotire.seconds() <= 2 && !isStopRequested()) {
                    motor_intake.setPower(0.3);
                }
                motor_intake.setPower(0);
                rotire.reset();


            }



    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition((int) (newLeftTarget + 2.244));
            rightBack.setTargetPosition((int) (newRightTarget + 2.244));

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy()) &&
                    (leftBack.isBusy() && rightBack.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Count per Inch: ", COUNTS_PER_INCH);
            telemetry.update();


            sleep(250);   // optional pause after each move.
        }
    }
}
