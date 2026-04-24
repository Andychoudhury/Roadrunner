package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop test test",group = "aLinear OpMode")

public class TeleopWithActions extends LinearOpMode {
    public void runOpMode(){
    // Example snippet to illustrate the logic (not complete code)
// Assumes you have a PID controller class (e.g., HeadingPIDController)
    HeadingPIDController headingPID = new HeadingPIDController(0,0); // Initialize with gains
    double targetHeading = Math.toRadians(0); // Example target heading (0 degrees)
    boolean headingLockActive = false;

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        drive.localizer.setPose(PoseStorage.currentPose);

        boolean aPressedLastFrame = false;

while (opModeIsActive()) {


        // Read pose and gamepad inputs
        Pose2d currentPose = drive.localizer.getPose();

        // Toggle heading lock mode with a button press
        if (gamepad1.a && !aPressedLastFrame) {
            headingLockActive = !headingLockActive;
            if (headingLockActive) {
                targetHeading = currentPose.heading.toDouble(); // Lock onto the current heading
            }
        }
        aPressedLastFrame = gamepad1.a;

        if (headingLockActive) {
            // Calculate the heading correction using the PID controller
            double headingCorrection = headingPID.update(targetHeading, currentPose.heading.toDouble());

            // Pass the correction as the rotational power (rx)
            // Adjust x and y inputs for normal movement (field-centric is common here)
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x),
                    headingCorrection // Use the PID output for rotation
            ));
        } else {
            // Normal driver control (field-centric code below for best results)
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                    input.x,
                    input.y),
                    -gamepad1.right_stick_x
            ));
        }

        // Update localization
        drive.localizer.update();

        // Telemetry updates
        telemetry.addData("Heading Lock Active", headingLockActive);
        telemetry.addData("Current Heading", Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.update();
    }
}}