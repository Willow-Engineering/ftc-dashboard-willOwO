package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Variables_MO")
public class Variables_MO extends OpMode {


    int myVar = 0;



    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
            if(gamepad1.a) {
                myVar += 5;
            }

            float yPos = gamepad1.left_stick_y;

            telemetry.addData("myVar is now at: ", myVar);
            telemetry.addData("A Button: ", gamepad1.a);
            telemetry.addData("Left Stick Y: ", yPos);
            telemetry.update();
    }

}

