package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Teleop Test", group="aLinear OpMode")

public class TeleopTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx rightShooter = null;
    private DcMotorEx leftShooter = null;

    private CRServo servo = null;

    public void runOpMode() {

        initAprilTag();
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

        boolean fieldCentric = false;

        boolean red = true;
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        // Set your initial pose to x: 10, y: 10, facing 90 degrees


        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.left_trigger>0) {
                red = true;
            }
            if (gamepad1.right_trigger>0) {
                red = false;
            }
            Pose2d pose = telemetryAprilTag(red);
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.updatePoseEstimate();

            // Retrieve your pose
            Pose2d myPose = drive.localizer.getPose();

            telemetry.addData("x", myPose.position.x);
            telemetry.addData("y", myPose.position.y);
            telemetry.addData("heading", Math.toDegrees(myPose.heading.toDouble()));

            if (gamepad1.left_bumper) {
                if (red) {
                    double heading = 45 + atan(pose.position.y+24/pose.position.x);
                    drive.localizer.setPose(new Pose2d(0,0,heading));
                }
                drive.localizer.setPose(new Pose2d(0,0,0));
            }

            // Insert whatever teleop code you're using
            double max;

            if (gamepad1.dpad_left) {
                fieldCentric = true;
            }
            if (gamepad1.dpad_right) {
                fieldCentric = false;
            }

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
            double h = (myPose.heading.toDouble())%360;
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double rotX = lateral * Math.cos(-h) - axial * Math.sin(-h);
            double rotY = lateral * Math.sin(-h) + axial * Math.cos(-h);


            double frontLeftPower = (fieldCentric) ?rotY + rotX + yaw : axial + lateral + yaw;
            double backLeftPower = (fieldCentric) ? rotY - rotX + yaw : axial - lateral + yaw;
            double frontRightPower = (fieldCentric) ? rotY - rotX - yaw : axial - lateral - yaw;
            double backRightPower = (fieldCentric) ? rotY + rotX - yaw : axial + lateral - yaw;

//            double frontLeftPower  = axial + lateral + yaw;
//            double frontRightPower = axial - lateral - yaw;
//            double backLeftPower   = axial - lateral + yaw;
//            double backRightPower  = axial + lateral - yaw;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.


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
            telemetry.addData("Field Centric",fieldCentric);
            telemetry.update();
        }
        visionPortal.close();
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private Pose2d telemetryAprilTag(boolean red) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (detection.id == 24 && red) {
                    telemetry.addLine("red");
                    telemetry.addData("Pose x", detection.ftcPose.x);
                    telemetry.addData("Pose y", detection.ftcPose.y);
                    telemetry.addData("Pose h", detection.ftcPose.bearing);
                    return new Pose2d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.bearing);
                }
                if (detection.id == 20 && !red) {
                    telemetry.addLine("blue");
                    telemetry.addData("Pose x", detection.ftcPose.x);
                    telemetry.addData("Pose y", detection.ftcPose.y);
                    telemetry.addData("Pose h", detection.ftcPose.bearing);
                    return new Pose2d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.bearing);

                }
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }// end for() loop
        return new Pose2d(0,0,0);
        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}