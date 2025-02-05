package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.thewaverlyschool.WaverlyGamepad;

@Autonomous
public class RavensAutonomous extends OpMode {

    // Settings
    double COUNTS_PER_INCH = 56.6;
    int targetInches = 30 ;
    double power = 0.5;
    String dir = "back";

    double clawServoClosedPos = 0.4;
    double clawServoOpenPos = 0.2;

    int clawUpPos = 3450;

    // Hardware
    // Hardware
    WaverlyGamepad gp1;
    WaverlyGamepad gp2;

    // Wheels
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;

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

        fl = hardwareMap.dcMotor.get("frontL");
        bl = hardwareMap.dcMotor.get("backL");
        fr = hardwareMap.dcMotor.get("frontR");
        br = hardwareMap.dcMotor.get("backR");

        clawE = hardwareMap.dcMotor.get("clawE");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hangMotor = hardwareMap.dcMotor.get("hang");

        frontTouch = hardwareMap.touchSensor.get("frontTouch");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL.setDirection(Servo.Direction.REVERSE);
        clawL.setPosition(clawServoClosedPos);
        clawR.setPosition(clawServoClosedPos);
    }

    @Override
    public void init_loop() {
        gp1.readButtons();

        //direction
        if (gp1.dpadLeftPressed) {
            dir = "left";
        } else if (gp1.dpadRightPressed) {
            dir = "right";
        } else if (gp1.dpadUpPressed) {
            dir = "forward";
        } else if (gp1.dpadDownPressed) {
            dir = "back";
        }

        //power change
        if (gp1.leftTriggerPressed) {
            power = Math.max(power - 0.1, 0.1);
        } else if (gp1.rightTriggerPressed) {
            power = Math.min(power + 0.1, 1.0);
        }


        if (gp1.leftBumperPressed) {
            targetInches = Math.max(targetInches - 1, 1);
        } else if (gp1.rightBumperPressed) {
            targetInches++;
        }
        telemetry.addData("targetInches", targetInches);
        telemetry.addData("dir", dir);
        telemetry.addData("power", power);
        telemetry.update();
    }

    @Override
    public void loop() {

       // raiseClaw();

        // Drive forward
        int targetPosition = (int) (targetInches * COUNTS_PER_INCH);
        driveToPosition(targetPosition);
    }

    private void raiseClaw() {
        clawL.setPosition(clawServoClosedPos);
        clawR.setPosition(clawServoClosedPos);
        clawE.setTargetPosition(clawUpPos);
        clawE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawE.setPower(0.5);
    }

    private void lowerClaw() {
        clawE.setTargetPosition(0);
        clawE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawE.setPower(0.5);
    }


    private void driveToPosition(int targetPosition) {
        if (dir.equals("left")|| dir.equals("right")){
            targetPosition= (int) Math.round(targetPosition*1.05);
        }
        int flPos = targetPosition;
        int blPos = targetPosition;
        int frPos = targetPosition;
        int brPos = targetPosition;

        if (dir.equals("forward")) {
            flPos = flPos * -1;
            blPos = blPos * -1;
            frPos = frPos * -1;
            brPos = brPos * -1;
        } else if (dir.equals("right")) {
            flPos = flPos * -1;
            brPos = brPos * -1;
        } else if (dir.equals("left")) {
            frPos = frPos * -1;
            blPos = blPos * - 1;
        }

        fl.setTargetPosition(flPos);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(power);
        bl.setTargetPosition(blPos);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setPower(power);
        fr.setTargetPosition(frPos);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setPower(power);
        br.setTargetPosition(brPos);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setPower(power);
        telemetry.addData("fl", fl.getCurrentPosition());
        telemetry.addData("bl", bl.getCurrentPosition());
        telemetry.addData("fr", fr.getCurrentPosition());
        telemetry.addData("br", br.getCurrentPosition());
        telemetry.update();
    }

}