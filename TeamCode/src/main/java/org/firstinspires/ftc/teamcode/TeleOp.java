package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fl = hardwareMap.dcMotor.get("frontL");
        DcMotor bl = hardwareMap.dcMotor.get("backL");
        DcMotor fr = hardwareMap.dcMotor.get("frontR");
        DcMotor br = hardwareMap.dcMotor.get("backR");
        DcMotor extendMotor = hardwareMap.dcMotor.get("extend");
        DcMotor pivotMotor = hardwareMap.dcMotor.get("pivot");


        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //run without encoder doesn't mean it stops encoder
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        telemetry.addData("Greeting", "Hello Waverly Robotics students!!!! ヾ(≧▽ ≦*)o ❤️ >.<");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y * -1; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 1; // Without negatives, controls are flipped

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);

            double extend = gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;

            extendMotor.setPower(extend);
            pivotMotor.setPower(pivot);


            telemetry.addData("leftY", gamepad1.left_stick_y);
            telemetry.addData("leftX", gamepad1.left_stick_x);
            telemetry.addData("rightX", gamepad1.right_stick_x);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("extendPower", extend);
            telemetry.addData("pivotPower", pivot);
            telemetry.addData("flPos",fl.getCurrentPosition());
            telemetry.addData("blPos",fl.getCurrentPosition());
            telemetry.addData("frPos",fl.getCurrentPosition());
            telemetry.addData("brPos",fl.getCurrentPosition());
            telemetry.update();

        }
    }
}
