package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.thewaverlyschool.WaverlyGamepad;


@TeleOp
public class ServoTest extends OpMode {

    // Settings
    double clawPower = 1;

    // Hardware
    WaverlyGamepad gp1;
    CRServo clawWheel;

    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);
        clawWheel = hardwareMap.crservo.get("clawWheel");

        telemetry.addData("Greeting", "Hello Waverly Robotics students!!!! ヾ(≧▽ ≦*)o ❤️ >.<");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        syncGamepads();
    }

    @Override
    public void loop() {
        syncGamepads();

        if (gp1.dpadUpPressed) {
            clawPower = Math.min(1, clawPower + 0.1);
        } else if (gp1.dpadDownPressed) {
            clawPower = Math.max(0.1, clawPower - 0.1);
        }

        if (gamepad1.right_bumper) {
            clawWheel.setDirection(CRServo.Direction.FORWARD);
            clawWheel.setPower(clawPower);
        } else if (gamepad1.left_bumper) {
            clawWheel.setDirection(CRServo.Direction.REVERSE);
            clawWheel.setPower(clawPower);
        } else {
            clawWheel.setPower(0);
        }

        //-------------------------------
        // Telemetry
        //-------------------------------
        telemetry.addData("clawPower", clawPower);
        telemetry.update();
    }

    private void syncGamepads() {
        gp1.readButtons();
    }

}
