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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: Auto Drive By Time",group="Pushbot")
//@Disabled

public class PushbotAutoDriveByTimeLinear extends LinearOpMode {

    /* Declare OpMode members. */
//    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor linearSlide;
    DcMotor linearRetract;
    Servo servoMarker;

    static final double     FORWARD_SPEED = 0.4;
    static final double     FORWARD_SPEED2 = 0.05;
    static final double     TURN_SPEED    = 0.5;
    static final double     STOP_SPEED = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);
//        leftMotor = hardwareMap.dcMotor.get("left_motor");
//        rightMotor = hardwareMap.dcMotor.get("right_motor");
        linearSlide = hardwareMap.dcMotor.get("linearSlide");
        linearRetract = hardwareMap.dcMotor.get("linearRetract");
        servoMarker = hardwareMap.servo.get("servoMarker");
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //team marker code
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            servoMarker.setPosition(0.5);
            telemetry.addData("TeamMarker", "Servo moved");    //
            telemetry.update();
        }

//        //extend the arm
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.7)) {
//            linearSlide.setPower(-FORWARD_SPEED);
//            linearRetract.setPower(-FORWARD_SPEED2);
//            telemetry.addData("Path","Linear 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            linearSlide.setPower(STOP_SPEED);
//            linearRetract.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for .5 seconds
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < .5)) {
//            leftMotor.setPower(-TURN_SPEED);
////            rightMotor.setPower(TURN_SPEED);
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftMotor.setPower(STOP_SPEED);
//            rightMotor.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for .5 seconds
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < .5)) {
//            leftMotor.setPower(TURN_SPEED);
////            rightMotor.setPower(-TURN_SPEED);
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftMotor.setPower(STOP_SPEED);
//            rightMotor.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//
////        // Step 3:  Drive forward for 1 Second
////        runtime.reset();
////        while (opModeIsActive() && (runtime.seconds() < .5)) {
////            leftMotor.setPower(-FORWARD_SPEED);
////            rightMotor.setPower(-FORWARD_SPEED);
////            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
////            telemetry.update();
////        }
////        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
////            leftMotor.setPower(STOP_SPEED);
////            rightMotor.setPower(STOP_SPEED);
////            telemetry.addData("Motor", "Stopped");    //
////            telemetry.update();
////        }
////
////        // Step 2:  Spin right for .3 seconds
////        runtime.reset();
////        while (opModeIsActive() && (runtime.seconds() < .1)) {
////            leftMotor.setPower(-TURN_SPEED);
////            rightMotor.setPower(TURN_SPEED);
////            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
////            telemetry.update();
////        }
////        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
////            leftMotor.setPower(STOP_SPEED);
////            rightMotor.setPower(STOP_SPEED);
////            telemetry.addData("Motor", "Stopped");    //
////            telemetry.update();
////        }
//
//        // Drive forward for 2 seconds
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
//            leftMotor.setPower(-FORWARD_SPEED);
//            rightMotor.setPower(-FORWARD_SPEED);
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftMotor.setPower(STOP_SPEED);
//            rightMotor.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//
//        //team marker code
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            servoMarker.setPosition(1);
//            telemetry.addData("TeamMarker", "Servo moved");    //
//            telemetry.update();
//        }
//
//        //Drive Backwards for .5 Second
//        runtime.reset();
//        leftMotor.setPower(FORWARD_SPEED);
//        rightMotor.setPower(FORWARD_SPEED);
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftMotor.setPower(STOP_SPEED);
//            rightMotor.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//
//        //team marker code
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            servoMarker.setPosition(0);
//            telemetry.addData("TeamMarker", "Servo moved");    //
//            telemetry.update();
//        }
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftMotor.setPower(STOP_SPEED);
//            rightMotor.setPower(STOP_SPEED);
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
////
////        //retract the arm
////        runtime.reset();
////        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
////            linearSlide.setPower(FORWARD_SPEED2);
////            linearRetract.setPower(FORWARD_SPEED);
////            telemetry.addData("Path", "Linear 1: %2.5f S Elapsed", runtime.seconds());
////            telemetry.update();
////        }
////        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
////            linearRetract.setPower(STOP_SPEED);
////            linearRetract.setPower(STOP_SPEED);
////            telemetry.addData("Motor", "Stopped");    //
////            telemetry.update();
////        }
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
    }
}
