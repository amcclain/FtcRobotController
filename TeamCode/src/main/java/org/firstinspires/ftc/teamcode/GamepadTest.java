package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.thewaverlyschool.WaverlyGamepad;

@TeleOp
public class GamepadTest extends OpMode {

    WaverlyGamepad gp1;
    int aPresses = 0;
    int leftPresses = 0;
    int rightPreses = 0;

    public void init() {
        gp1 = new WaverlyGamepad(gamepad1);
    }

    @Override
    public void init_loop() {
        gp1.readButtons();
        if (gp1.aPressed) aPresses++;
        if (gp1.dpadLeftPressed) leftPresses++;
        if (gp1.dpadRightPressed) rightPreses++;

        telemetry.addLine("Up/Down | ")
                .addData("a", gp1.a ? "DOWN" : "UP")
                .addData("left", gp1.dpadLeft ? "DOWN" : "UP")
                .addData("right", gp1.dpadRight ? "DOWN" : "UP");

        telemetry.addLine("Presses | ")
                .addData("a", aPresses)
                .addData("left", leftPresses)
                .addData("right", rightPreses);

        telemetry.update();
    }

    @Override
    public void loop() {
        gp1.readButtons();
        telemetry.update();
        telemetry.update();
    }

}