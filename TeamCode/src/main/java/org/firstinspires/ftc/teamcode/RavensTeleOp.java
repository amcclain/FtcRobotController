package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.thewaverlyschool.WaverlyGamepad;


@TeleOp
public class RavensTeleOp extends OpMode {

    // Settings
    Boolean flipY = false;
    Boolean flipX = false;
    Boolean flipRx = false;
    boolean slomo = false;

    double hangPower = 0.5;
    double clawServoClosedPos = 0.0;
    double clawServoOpenPos = 0.5;

    // Hardware
    WaverlyGamepad gp1;
    WaverlyGamepad gp2;

    // Wheels
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;

    // Claw arm
    CRServo clawExtendServo;
    Servo clawL;
    Servo clawR;

    // Hang arm
    DcMotor hangMotor;

    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);
        gp2 = new WaverlyGamepad(gamepad2);

        fl = hardwareMap.dcMotor.get("frontL");
        bl = hardwareMap.dcMotor.get("backL");
        fr = hardwareMap.dcMotor.get("frontR");
        br = hardwareMap.dcMotor.get("backR");

        clawExtendServo = hardwareMap.crservo.get("armExtend");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hangMotor = hardwareMap.dcMotor.get("hang");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawL.setDirection(Servo.Direction.REVERSE);
        clawL.setPosition(clawServoClosedPos);
        clawR.setPosition(clawServoClosedPos);

        telemetry.addData("Greeting", "Hello Waverly Robotics students!!!! ヾ(≧▽ ≦*)o ❤️ >.<");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        syncGamepads();
        adjustSettings(true);

        // Changes to closed pos take effect during init - check if this is allowed
        clawL.setPosition(clawServoClosedPos);
        clawR.setPosition(clawServoClosedPos);
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
        // Claw arm
        //-------------------------------
        double clawExtendStick = gamepad2.left_stick_y;
        if (clawExtendStick > 0) {
            clawExtendServo.setPower(1);
        } else if (clawExtendStick < 0) {
            clawExtendServo.setPower(-1);
        } else {
            clawExtendServo.setPower(0);
        }

        if (gp2.rightBumperPressed) {
            clawL.setPosition(clawServoOpenPos);
            clawR.setPosition(clawServoOpenPos);
        } else if (gp2.leftBumperPressed) {
            clawL.setPosition(clawServoClosedPos);
            clawR.setPosition(clawServoClosedPos);
        }


        //-------------------------------
        // Hang
        //-------------------------------
        double hangExtendStick = gamepad2.right_stick_y;
        if (hangExtendStick > 0) {
            hangMotor.setPower(hangPower);
        } else if (hangExtendStick < 0) {
            hangMotor.setPower(-hangPower);
        } else {
            hangMotor.setPower(0);
        }


        //-------------------------------
        // Telemetry
        //-------------------------------
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("rightX", gamepad1.right_stick_x);
        telemetry.addLine("=============POWER================");
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);
        telemetry.addData("hangPower", hangPower);
        telemetry.addLine("==============POSITIONS================");
        telemetry.addData("flPos", fl.getCurrentPosition());
        telemetry.addData("blPos", fl.getCurrentPosition());
        telemetry.addData("frPos", fl.getCurrentPosition());
        telemetry.addData("brPos", fl.getCurrentPosition());
        telemetry.addData("clawLPos", clawL.getPosition());
        telemetry.addData("clawRPos", clawR.getPosition());
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

        // Claw servo position
        if (gp2.dpadRightPressed) {
            clawServoOpenPos = Math.min(1, clawServoOpenPos + 0.1);
        } else if (gp2.dpadLeftPressed) {
            clawServoOpenPos = Math.max(-1, clawServoOpenPos - 0.1);
        }
        if (gp2.dpadUpPressed) {
            clawServoClosedPos = Math.min(1, clawServoClosedPos + 0.1);
        } else if (gp2.dpadDownPressed) {
            clawServoClosedPos = Math.max(-1, clawServoClosedPos - 0.1);
        }

        // Hang power
        if (gp2.rightTriggerPressed) {
            hangPower = Math.min(1, hangPower + 0.1);
        } else if (gp2.leftTriggerPressed) {
            hangPower = Math.max(-1, hangPower - 0.1);
        }

        telemetry.addData("Flip left/right (dpad L)", flipRx);
        telemetry.addData("Flip fwd/rev (dpad U)", flipY);
        telemetry.addData("Flip strafe (dpad R)", flipX);
        telemetry.addData("SlowMo (dpad D)", slomo);
        telemetry.addData("Claw servo closed (dpad u/d)", clawServoClosedPos);
        telemetry.addData("Claw servo open (dpad l/r)", clawServoOpenPos);
        telemetry.addData("Hang Power (trigger l/r)", hangPower);
    }

}
