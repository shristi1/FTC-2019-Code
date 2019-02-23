package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OldCode extends LinearOpMode {
    //    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor linearSlide;
    private DcMotor linearRetract;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
//        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        motorFL = hardwareMap.get(DcMotor.class, "motor_fl");
        motorFR = hardwareMap.get(DcMotor.class, "motor_fr");
        motorBL = hardwareMap.get(DcMotor.class, "motor_bl");
        motorBR = hardwareMap.get(DcMotor.class, "motor_br");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearRetract = hardwareMap.get(DcMotor.class, "linearRetract");

//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;

        while (opModeIsActive()) {
            //
            double G1rightStickY = -gamepad1.right_stick_y;
            double G1leftStickY = -gamepad1.left_stick_y;
            boolean G1rightBumper = gamepad1.right_bumper;
            boolean G1leftBumper = gamepad1.left_bumper;

            if (G1rightBumper) {
                //motor front, left
                this.motorFL.setPower(1);

                //motor front, right
                this.motorFR.setPower(-1);

                //motor back, right
                this.motorBR.setPower(-1);

                //motor back, left
                this.motorBL.setPower(1);

            }
            else if (G1leftBumper){
                //motor front, left
                this.motorFL.setPower(-1);

                //motor front, right
                this.motorFR.setPower(1);

                //motor back, right
                this.motorBR.setPower(1);

                //motor back, left
                this.motorBL.setPower(-1);
            }
            else {
                //motor front, left
                this.motorFL.setPower(G1leftStickY);

                //motor front, right
                this.motorFR.setPower(G1leftStickY);

                //motor back, right
                this.motorBR.setPower(-G1rightStickY);

                //motor back, left
                this.motorBL.setPower(-G1rightStickY);
            }

//            //left motor
//            tgtPower = -this.gamepad1.left_stick_y;
//            leftMotor.setPower(tgtPower);
//
//            //right motor
//            tgtPower = this.gamepad1.right_stick_y;
//            rightMotor.setPower(tgtPower);

//            //move up linear slide RT
//            tgtPower = this.gamepad1.right_trigger;
//            linearSlide.setPower(tgtPower);
//
//            //move up linear slide LT
//            tgtPower = this.gamepad1.left_trigger;
//            linearRetract.setPower(tgtPower);

//            //move down linear slide
//            if (this.gamepad1.left_bumper && this.gamepad1.right_bumper){
//                telemetry.addData("Status", "in if statement");
//                telemetry.update();
//
//                //move up linear slide RT
//                tgtPower = -this.gamepad1.right_trigger;
//                linearSlide.setPower(tgtPower);
//                telemetry.addData("Status", "RT");
//                telemetry.update();
//
//                //move up linear slide LT
//                tgtPower = -this.gamepad1.left_trigger;
//                linearRetract.setPower(tgtPower);
//                telemetry.addData("Status", "LT");
//                telemetry.update();uu
//
//                //move up linear slide RT
//                tgtPower = this.gamepad1.right_trigger;
//                linearSlide.setPower(tgtPower);
//
//                //move up linear slide LT
//                tgtPower = this.gamepad1.left_trigger;
//                linearRetract.setPower(tgtPower);
//            }

//            retract linear slide
            if (this.gamepad1.left_bumper && this.gamepad1.right_bumper) {
                telemetry.addData("Status", "in if statement");
                telemetry.update();
                //Retract the arm
                linearSlide.setPower(-tgtPower);
                linearRetract.setPower(tgtPower);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
