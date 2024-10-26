package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="TestAuto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor hex1 = hardwareMap.dcMotor.get("hex1");
        hex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  hex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("hex1",hex1.getCurrentPosition());
        telemetry.update();


        hex1.setTargetPosition(0);

        int secondsToRun = 3;
        boolean didRun = false;


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            int ticks = hex1.getCurrentPosition();
            hex1.setTargetPosition(1000);
            hex1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hex1.setPower(.1);
            telemetry.addData("hex1",hex1.getCurrentPosition());
            telemetry.update();

//            if (!didRun) {
//                hex1.setPower(1);
//                Thread.sleep(secondsToRun * 1000);
//                hex1.setPower(0);
//                didRun = true;



           // }




        }
    }
}
