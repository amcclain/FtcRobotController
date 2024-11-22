 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.thewaverlyschool.WaverlyGamepad;


@TeleOp
public class MotorTest extends OpMode {

    // Settings
    double motorPower = 1;

    // Hardware
    WaverlyGamepad gp1;
    DcMotor motor;

    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);
        motor = hardwareMap.dcMotor.get("motor");

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
            motorPower = Math.min(1, motorPower + 0.1);
        } else if (gp1.dpadDownPressed) {
            motorPower = Math.max(0.1, motorPower - 0.1);
        }

        if (gamepad1.right_bumper) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(motorPower);
        } else if (gamepad1.left_bumper) {
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setPower(motorPower);
        } else {
            motor.setPower(0);
        }

        //-------------------------------
        // Telemetry
        //-------------------------------
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
    }

    private void syncGamepads() {
        gp1.readButtons();
    }

}
