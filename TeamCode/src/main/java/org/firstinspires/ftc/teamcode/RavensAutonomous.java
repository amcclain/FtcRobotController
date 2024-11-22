package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.thewaverlyschool.WaverlyGamepad;

@Autonomous
public class RavensAutonomous extends OpMode {

    // Settings
    double COUNTS_PER_INCH = 56.6;
    int targetInches = 36;
    double power = 0.5;

    // Hardware
    WaverlyGamepad gp1;
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;

    public void init() {
        gp1 = new WaverlyGamepad(gamepad1);
        fl = hardwareMap.dcMotor.get("frontL");
        bl = hardwareMap.dcMotor.get("backL");
        fr = hardwareMap.dcMotor.get("frontR");
        br = hardwareMap.dcMotor.get("backR");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void init_loop() {
        gp1.readButtons();

        if (gp1.dpadLeftPressed) {
            targetInches--;
        } else if (gp1.dpadRightPressed ) {
            targetInches++;
        }
        telemetry.addData("targetInches", targetInches);
        telemetry.update();
    }

    @Override
    public void loop() {
        int targetPosition = (int) (targetInches*COUNTS_PER_INCH);
        fl.setTargetPosition(targetPosition);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(power);
        bl.setTargetPosition(targetPosition);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setPower(power);
        fr.setTargetPosition(targetPosition);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setPower(power);
        br.setTargetPosition(targetPosition);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setPower(power);
        telemetry.addData("fl", fl.getCurrentPosition());
        telemetry.addData("bl", bl.getCurrentPosition());
        telemetry.addData("fr", fr.getCurrentPosition());
        telemetry.addData("br", br.getCurrentPosition());
        telemetry.update();

    }

}