package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CrabBot Demo 2023-2024")
public class CrabBotDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize connected hardware
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        Servo tailServo = hardwareMap.get(Servo.class, "tailServo");
        // lights = hardwareMap.get(Servo.class, "lights");

        // Flip left-side motors so right and left spin the same way.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset tail position to center
        tailServo.setPosition(0.5);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //-----------------------------
                // Tail Controls
                //-----------------------------
                if (gamepad1.right_bumper && gamepad1.left_bumper) {
                    tailServo.setPosition(0.5);
                } else {
                    if (gamepad1.right_bumper) {
                        tailServo.setPosition(0.2);
                    }
                    if (gamepad1.left_bumper) {
                        tailServo.setPosition(0.8);
                    }
                }


                //-----------------------------
                // Primary Drive / Steering
                //-----------------------------

                // Y controls forward/backward speed
                float leftPower = -gamepad1.left_stick_y;
                float rightPower = -gamepad1.left_stick_y;


                // X controls turning - sign indicates direction
                float x = gamepad1.left_stick_x;
                float turnFactor = Math.abs(x);


                if (-gamepad1.left_stick_y>0.1) {
                    telemetry.addData(">",1);
                    if (x < 0) {
                        // Turn left - right wheels must spin faster
                        rightPower = rightPower + turnFactor;
                    } else if (x > 0) {
                        // Turn right - left wheels faster
                        leftPower = leftPower + turnFactor;
                    }
                } else if (-gamepad1.left_stick_y<-0.1){
                    telemetry.addData("<",1);
                    if (x < 0) {
                        // Turn left - right wheels must spin faster
                        rightPower = rightPower + turnFactor;
                    } else if (x > 0) {
                        // Turn right - left wheels faster
                        leftPower = leftPower + turnFactor;
                    }
                } else if (-gamepad1.left_stick_y==0){
                    telemetry.addData("=",1);
                    if (x < 0) {
                        // Turn left - right wheels must spin faster
                        rightPower = rightPower + turnFactor;
                    } else if (x > 0) {
                        // Turn right - left wheels faster
                        leftPower = leftPower + turnFactor;
                    }
                }

                frontRight.setPower(rightPower);
                backRight.setPower(rightPower);
                frontLeft.setPower(leftPower);
                backLeft.setPower(leftPower);

                telemetry.addData("rightPower", rightPower);
                telemetry.addData("leftPower", leftPower);
                telemetry.addData("y", gamepad1.left_stick_y);
                telemetry.addData("x", x);
                telemetry.update();
            }
        }
    }
}
