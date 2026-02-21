/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Back Red Auto")
public class BackRedAuto extends LinearOpMode {

    DcMotor frontleft = null;
    DcMotor frontright = null;
    DcMotor backleft = null;
    DcMotor backright = null;
    DcMotorEx leftShooter = null;
    DcMotorEx rightShooter = null;
    CRServo servo = null;
    ElapsedTime runtime = new ElapsedTime();


    Integer cpr = 30; //counts per rotation, see motor specs
    Double gearRatio = 19.2; //see motor specs and multiply by any gear reductions in drivetrain
    Double diameter = 4.09449; //wheel diameter
    Double cpi = (cpr * gearRatio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))

    Double bias = 0.85;//adjust error by dividing intended fwd motion by actual
    Double meccyBias = 1.85;//change to adjust only strafing movement error
    Double turnBias = 0.249; //change to adjust turing error
    Boolean exit = false; //keeps auto process from quitting until mpvement is finished


    public void runOpMode(){
        //edit motor map names to match robot config
        frontleft = hardwareMap.dcMotor.get("FrontLeft");
        frontright = hardwareMap.dcMotor.get("FrontRight");
        backleft = hardwareMap.dcMotor.get("BackLeft");
        backright = hardwareMap.dcMotor.get("BackRight");
        leftShooter = hardwareMap.get(DcMotorEx.class,"LeftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class,"RightShooter");
        servo = hardwareMap.crservo.get("servo");


        //adjust for motor mounting as needed
        frontright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


        //POSITIVE STRAFE IS LEFT

        //NEGATIVE TURN IS LEFT, POSITIVE TURN IS RIGHT

        //input list of movement commands.  Can tune sleep between commands to minimize wait between moves or ensure complete motion.
        shoot(850);
        turnWithEncoder(-22,.4);
        sleep(200);
        strafeToPosition(-7,.4);



        //
        // strafeToPosition();







    }


    //







    /*


    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */

    public void moveToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * bias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(-speed);
        backleft.setPower(-speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            telemetry.addData("fl motor",frontleft.getCurrentPosition());
            telemetry.addData("fr motor",frontright.getCurrentPosition());
            telemetry.addData("bl motor",backleft.getCurrentPosition());
            telemetry.addData("br motor",backright.getCurrentPosition());
            telemetry.update();
            if (exit) {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        //return;
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(-inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            if (exit) {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        //return;
    }

    /*
       This function uses the encoders to turn left or right.
       Negative input for angle results in counterclockwise rotation.
        */
    public void turnWithEncoder(double angle, double speed) {
        //
        int move = (int) (Math.round(angle * cpi * turnBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() - move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);


        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            if (exit) {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }

        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        //return;
    }

    public void shoot(int velocity) {
        runtime.reset();
        int skib = 0;
        int x = 3000;
        while (opModeIsActive() && runtime.milliseconds()<11500+x) {
            leftShooter.setVelocity(velocity);
            rightShooter.setVelocity(velocity);
            if (runtime.milliseconds()>7000+x && runtime.milliseconds()<8200+x) {
                servo.setPower(1);
                skib = 1;
            }
            if (runtime.milliseconds()> 8200+x && runtime.milliseconds()<9200+x) {
                servo.setPower(0);
            }
            if (runtime.milliseconds()>9200+x && runtime.milliseconds()<9500+x) {
                servo.setPower(1);
                skib = 2;
            }
            if (runtime.milliseconds()>9500+x && runtime.milliseconds()<10500+x) {
                servo.setPower(0);
            }
            if (runtime.milliseconds()>10500+x && runtime.milliseconds()<11500+x) {
                servo.setPower(1);
                skib = 3;
            }
            telemetry.addData("number: ",skib);
            telemetry.addData("Velocity",leftShooter.getVelocity());
            telemetry.addData("runtime",runtime.seconds());
            telemetry.update();
        }
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }




}