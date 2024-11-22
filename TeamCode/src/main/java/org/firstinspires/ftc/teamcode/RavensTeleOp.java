package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.thewaverlyschool.WaverlyGamepad;


@TeleOp
public class RavensTeleOp extends OpMode {

    // Settings
    Boolean flipY = false;
    Boolean flipX = false;
    Boolean flipRx = false;
    boolean slomo = false;
    double pivotPower = 1;

    // Hardware
    WaverlyGamepad gp1;
    WaverlyGamepad gp2;
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor extendMotor;
    DcMotor pivotMotor;
    CRServo clawWheel;

    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);
        gp2 = new WaverlyGamepad(gamepad2);
        fl = hardwareMap.dcMotor.get("frontL");
        bl = hardwareMap.dcMotor.get("backL");
        fr = hardwareMap.dcMotor.get("frontR");
        br = hardwareMap.dcMotor.get("backR");
        extendMotor = hardwareMap.dcMotor.get("extend");
        pivotMotor = hardwareMap.dcMotor.get("pivot");
        clawWheel = hardwareMap.crservo.get("clawWheel");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //run without encoder doesn't mean it stops encoder
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Greeting", "Hello Waverly Robotics students!!!! ヾ(≧▽ ≦*)o ❤️ >.<");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        syncGamepads();
        adjustSettings(true);
    }

    @Override
    public void loop() {
        syncGamepads();
        adjustSettings(false);

        //-------------------------------
        // Main motor drive
        //-------------------------------
        double y = gamepad1.left_stick_y;  // forward/backward
        double x = gamepad1.left_stick_x * -1.1;
        double rx = gamepad1.right_stick_x * -1;  // left/right turning

        if (flipY) y = y * -1;
        if (flipX) x = x * -1;
        if (flipRx) rx = rx * -1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (slomo) denominator = denominator * 2;

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);


        //-------------------------------
        // Arm pivot
        //-------------------------------
        double pivotStick = gamepad2.right_stick_y;
        if (pivotStick > 0) {
            pivotMotor.setPower(pivotPower);
        } else if (pivotStick < 0) {
            pivotMotor.setPower(-pivotPower);
        } else {
            pivotMotor.setPower(0);
        }


        //-------------------------------
        // Arm extend
        //-------------------------------
        double extendPower = gamepad2.left_stick_y;
        extendMotor.setPower(extendPower);


        //-------------------------------
        // Claw
        //-------------------------------
        double clawPower = 0;
        if (gamepad2.right_bumper) {
            clawPower = 1;
            clawWheel.setDirection(CRServo.Direction.FORWARD);
        } else if (gamepad2.left_bumper) {
            clawPower =  1;
            clawWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        clawWheel.setPower(clawPower);


        //-------------------------------
        // Telemetry
        //-------------------------------
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("rightX", gamepad1.right_stick_x);
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);
        telemetry.addData("extendPower", extendPower);
        telemetry.addData("flPos", fl.getCurrentPosition());
        telemetry.addData("blPos", fl.getCurrentPosition());
        telemetry.addData("frPos", fl.getCurrentPosition());
        telemetry.addData("brPos", fl.getCurrentPosition());
        telemetry.addData("clawPower", clawPower);
        telemetry.addData("pivotPos", pivotMotor.getCurrentPosition());
        telemetry.addData("extendPos", extendMotor.getCurrentPosition());

        telemetry.update();
    }

    private void syncGamepads() {
        gp1.readButtons();
        gp2.readButtons();
    }

    private void adjustSettings(boolean allowFlip) {
        // Drive flip
        if (allowFlip) {
            if (gp1.dpadLeftPressed) flipRx = !flipRx;
            if (gp1.dpadUpPressed) flipY = !flipY;
            if (gp1.dpadRightPressed) flipX = !flipX;
        }

        // Drive power
        if (gp1.dpadDownPressed) slomo = !slomo;

        // Arm power
        if (gp2.dpadUpPressed) {
            pivotPower = Math.min(1, pivotPower + 0.1);
        } else if (gp2.dpadDownPressed) {
            pivotPower = Math.max(0.1, pivotPower - 0.1);
        }

        telemetry.addData("Flip left/right", flipRx);
        telemetry.addData("Flip fwd/rev", flipY);
        telemetry.addData("Flip strafe", flipX);
        telemetry.addData("SlowMo", slomo);
        telemetry.addData("Pivot Power", pivotPower);
    }

}
