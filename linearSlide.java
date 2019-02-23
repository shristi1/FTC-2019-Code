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
/*
    Strafe left: BL = - BR = -
    Strafe right: FL = - FR = -
    Drive Back : FR = - BR = -
    Drive Forward: Fl = - Bl = -
    Turn Left: FL = - or all positive
    Turn Right = all negative
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="linearSlide", group="auto")
//@Disabled

public class linearSlide extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;
    DcMotor linearSlide;
    DcMotor linearRetract;
    Servo servoMarker;

    static final double     FORWARD_SPEED = 0.4;
    static final double     FORWARD_SPEED2 = 0.05;
    static final double     STOP_SPEED = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        linearSlide = hardwareMap.dcMotor.get("linearSlide");
        linearRetract = hardwareMap.dcMotor.get("linearRetract");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //extend the arm
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.7)) {
            linearSlide.setPower(FORWARD_SPEED);
            linearRetract.setPower(FORWARD_SPEED);
            telemetry.addData("Path", "Linear 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            linearSlide.setPower(STOP_SPEED);
            linearRetract.setPower(STOP_SPEED);
            telemetry.addData("Motor", "Stopped");    //
            telemetry.update();
        }


    }
}