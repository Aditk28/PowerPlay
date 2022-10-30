package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Hardware
    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;
    public Servo rightClaw;
    public Servo leftClaw;
    //otherVariables

    public boolean turtleMode = false;
    public static final double NORMAL_SPEED = 0.8;
    public static final double TURTLE_SPEED = 0.25;
    public double robotSpeed = NORMAL_SPEED;
    public double dropPosition = .5;
    public double pickPosition = .1;
    public double rotationSpeed = .75;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //clawMotor
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                rightClaw.setPosition(pickPosition);
                leftClaw.setPosition(pickPosition);
            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                rightClaw.setPosition(dropPosition);
                leftClaw.setPosition(dropPosition);
            }

            //Lift
            if (gamepad2.left_trigger != 0 || gamepad1.left_trigger != 0 && liftMotor.getCurrentPosition() >= 0) {
                liftMotor.setPower(-0.45);
            } else if (gamepad2.right_trigger != 0 || gamepad1.right_trigger != 0 && liftMotor.getCurrentPosition() <= 2050) {
                if (liftMotor.getCurrentPosition() >= 1500) {
                    liftMotor.setPower(0.3);
                }
                liftMotor.setPower(0.85);
            } else {
                liftMotor.setPower(0.0);
                if ((gamepad2.left_trigger == 0 && gamepad1.left_trigger == 0) && (!gamepad1.b && !gamepad2.b) && liftMotor.getCurrentPosition() >= 1000) {
                    liftMotor.setPower(0.01);
                } else {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }

//            //lift + bucket reset
            if ((gamepad2.b || gamepad1.b) && liftMotor.getCurrentPosition() > 0) {
                rightClaw.setPosition(dropPosition);
                leftClaw.setPosition(dropPosition);
                liftMotor.setPower(-0.50);
            }

            //turtleMode
            if (gamepad1.y && !turtleMode) {
                turtleMode = true;
                robotSpeed = TURTLE_SPEED;
            } else if (gamepad1.y && turtleMode) {
                turtleMode = false;
                robotSpeed = NORMAL_SPEED;
            }


            //movement
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * robotSpeed,
                            input.getY() * robotSpeed,
                            -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                    )
            );


//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * robotSpeed,
//                            -gamepad1.left_stick_x * robotSpeed,
//                            -gamepad1.right_stick_x * robotSpeed * .75
//                    )
//            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("DRIVE", "------------------------------------");
            telemetry.addData("Motor Power ", drive.getWheelVelocities());
            telemetry.addData("DriveMode: ", (turtleMode) ? ("turtleMode") : ("Normal"));
            telemetry.addData("Normal Speed: ", robotSpeed);
            telemetry.addData("OTHER", "------------------------------------");
            telemetry.addData("LiftMotor Position: ", liftMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}





