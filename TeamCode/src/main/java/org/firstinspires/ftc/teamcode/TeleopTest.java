package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop Test", group="aLinear OpMode")

public class TeleopTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx rightShooter = null;
    private DcMotorEx leftShooter = null;

    private CRServo servo = null;

    public void runOpMode() {
        // Insert whatever initialization your own code does
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        leftShooter = hardwareMap.get(DcMotorEx.class,"LeftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class,"RightShooter");
        servo = hardwareMap.get(CRServo.class,"servo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pid = new PIDFCoefficients(25,2.25,10.00,0);

        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pid);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pid);

        PIDFCoefficients pid1 = leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pid2 = rightShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        double speedMultiplier = 1;
        boolean highSpeed = false;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        // Set your initial pose to x: 10, y: 10, facing 90 degrees


        waitForStart();

        while(opModeIsActive()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.updatePoseEstimate();

            // Retrieve your pose
            Pose2d myPose = drive.localizer.getPose();

            telemetry.addData("x", myPose.position.x);
            telemetry.addData("y", myPose.position.y);
            telemetry.addData("heading", myPose.heading);

            // Insert whatever teleop code you're using
            double max;

            if (gamepad1.a) {
                speedMultiplier = 0.75;
            }
            if (gamepad1.b) {
                speedMultiplier = 0.50;
            }
            if (gamepad1.x) {
                speedMultiplier = 0.25;
            }
            if (gamepad1.right_bumper) {
                speedMultiplier = 1.00;
            }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            frontLeftPower *= speedMultiplier;
            frontRightPower *= speedMultiplier;
            backLeftPower *= speedMultiplier;
            backRightPower *= speedMultiplier;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > speedMultiplier) {
                frontLeftPower  /= (max/speedMultiplier);
                frontRightPower /= (max/speedMultiplier);
                backLeftPower   /= (max/speedMultiplier);
                backRightPower  /= (max/speedMultiplier);
            }

            /** Shooters **/
            double shooterPower;
            if (gamepad2.right_bumper) {
                shooterPower = 933;
            }
            else if (gamepad2.right_trigger>0) {
                shooterPower = 750;
            }
            else {
                shooterPower = 0;
            }

            if (gamepad2.left_bumper || (gamepad2.left_trigger>0)) {
                servo.setPower(1);
            }
            else {
                servo.setPower(0);
            }
            if (gamepad2.b) {
                servo.setPower(-1);
                shooterPower = -.45;
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            leftShooter.setVelocity(shooterPower);
            rightShooter.setVelocity(shooterPower);
//            leftShooter.setPower(shooterPower);
//            rightShooter.setPower(shooterPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Multiplier",speedMultiplier);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("High speed: ",highSpeed);
            telemetry.addData("Shooters","%4.2f",leftShooter.getVelocity());
            telemetry.addData("Servo Power",servo.getPower());
            telemetry.addData("PID","%4.2f, %4.2f, %4.2f, %4.2f", pid1.p, pid1.i, pid1.d, pid1.f);
            telemetry.update();
        }
    }
}