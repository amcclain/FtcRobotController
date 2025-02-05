package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.thewaverlyschool.WaverlyGamepad;


@TeleOp
public class RavensTeleOp extends OpMode {

    // Settings
    Boolean flipY = false;
    Boolean flipX = false;
    Boolean flipRx = false;
    boolean slomo = false;

    int clawUpPos = 3450;

    double clawServoClosedPos = 0.4;
    double clawServoOpenPos = 0.2;

    double hangPower = 1;

    // Hardware
    WaverlyGamepad gp1;
    WaverlyGamepad gp2;

    // Wheels
    DcMotorEx fl;
    DcMotorEx bl;
    DcMotorEx fr;
    DcMotorEx br;

    // Claw arm
    DcMotor clawE;
    Servo clawL;
    Servo clawR;

    // Hang arm
    DcMotor hangMotor;

    // Sensors
    TouchSensor frontTouch;

    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);
        gp2 = new WaverlyGamepad(gamepad2);

        fl = (DcMotorEx) hardwareMap.dcMotor.get("frontL");
        bl = (DcMotorEx) hardwareMap.dcMotor.get("backL");
        fr = (DcMotorEx) hardwareMap.dcMotor.get("frontR");
        br = (DcMotorEx) hardwareMap.dcMotor.get("backR");

        clawE = hardwareMap.dcMotor.get("clawE");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hangMotor = hardwareMap.dcMotor.get("hang");

        frontTouch = hardwareMap.touchSensor.get("frontTouch");


        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        clawE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawL.setDirection(Servo.Direction.REVERSE);
        clawL.setPosition(clawServoClosedPos);
        clawR.setPosition(clawServoClosedPos);

        telemetry.addData("Greeting", "Hello Waverly Robotics students..pleasesavemeohgodohgodthecodingmonsterisgoingtoeatme");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        syncGamepads();
        adjustSettings(true);

        // Changes to closed pos take effect during init - check if this is allowed
        clawL.setPosition(clawServoClosedPos);        clawR.setPosition(clawServoClosedPos);
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
        int clawEPos = clawE.getCurrentPosition();
        if (clawExtendStick > 0 && (clawEPos < clawUpPos || gp2.a)) {
            clawE.setPower(0.7);
        } else if (clawExtendStick < 0 && (clawEPos > 0 || gp2.a) ) {
            clawE.setPower(-0.7);
        } else {
            clawE.setPower(0);
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
        telemetry.addData("frontTouch", frontTouch.isPressed());
        telemetry.addLine("=============POWER================");
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);
        telemetry.addData("hangPower", hangPower);
        telemetry.addLine("==============POSITIONS================");
        telemetry.addData("flPos", fl.getCurrentPosition());
        telemetry.addData("blPos", bl.getCurrentPosition());
        telemetry.addData("frPos", fr.getCurrentPosition());
        telemetry.addData("brPos", br.getCurrentPosition());
        telemetry.addData("clawE", clawE.getCurrentPosition());
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
            clawServoOpenPos = Math.min(1, clawServoOpenPos + 0.05);
        } else if (gp2.dpadLeftPressed) {
            clawServoOpenPos = Math.max(-1, clawServoOpenPos - 0.05 );
        }
        if (gp2.dpadUpPressed) {
            clawServoClosedPos = Math.min(1, clawServoClosedPos + 0.05);
        } else if (gp2.dpadDownPressed) {
            clawServoClosedPos = Math.max(-1, clawServoClosedPos - 0.05);
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
private void setDriveMotorMode(DcMotor.RunMode mode){
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
}
}
