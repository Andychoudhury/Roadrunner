package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "BlueAutoNew", group = "Autonomous")
public class BlueAutoNew extends LinearOpMode {
    public class LeftShooter {
        private DcMotorEx leftShooter;

        public LeftShooter(HardwareMap hardwareMap) {
            leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooter");
            leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SpinUp implements Action {
            private boolean initialized = false;
            private final Action sleep = new SleepAction(7.0);

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftShooter.setVelocity(760);
                    initialized = true;
                }

                double vel = leftShooter.getVelocity();
                packet.put("shooterVelocity", vel);
                return sleep.run(packet);
            }
        }

        public Action spinUp() {
            return new SpinUp();
        }
    }

    public class RightShooter {
        private DcMotorEx rightShooter;

        public RightShooter(HardwareMap hardwareMap) {
            rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooter");
            rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SpinUp implements Action {
            private boolean initialized = false;
            private final Action sleep = new SleepAction(7.0);

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightShooter.setVelocity(760);
                    initialized = true;
                }

                double vel = rightShooter.getVelocity();
                packet.put("shooterVelocity", vel);
                return sleep.run(packet);
            }
        }

        public Action spinUp() {
            return new SpinUp();
        }
    }

    public class Srvo {
        private CRServo srvo;

        public Srvo(HardwareMap hardwareMap) {
            srvo = hardwareMap.get(CRServo.class, "servo");
        }

        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                srvo.setPower(1);
                return false;
            }
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                srvo.setPower(0);
                return false;
            }
        }
        
        public Action shoot() {
            return new Shoot();
        }
        
        public Action stop() {
            return new Stop();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Srvo srvo = new Srvo(hardwareMap);
        LeftShooter leftShooter = new LeftShooter(hardwareMap);
        RightShooter rightShooter = new RightShooter(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(8,65),Math.toRadians(95.3315));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        TrajectoryActionBuilder endStrafe = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(33,1));
        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Action EndStrafe = endStrafe.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            trajectoryActionChosen,
                            new ParallelAction(
                                leftShooter.spinUp(),
                                rightShooter.spinUp()
                            )
                        ),
                        srvo.shoot(),
                        new SleepAction(1),
                        srvo.stop(),
                        new SleepAction(1),
                        srvo.shoot(),
                        new SleepAction(.3),
                        srvo.stop(),
                        new SleepAction(1),
                        srvo.shoot(),
                        new SleepAction(1),
                        EndStrafe
                )
        );
    }
}