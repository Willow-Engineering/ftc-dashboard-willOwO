package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class turbo_modeSM extends OpMode {
    @Override
    public void init() {
    }
double button_pressed;
    @Override
    public void loop() {
        double fwdSpeed = gamepad1.left_stick_y;
        double ySpeed = gamepad1.left_stick_y;
        double xSpeed = gamepad1.left_stick_x;


        //turbo mode
        if (gamepad1.a) {
            fwdSpeed *= 0.5;
            fwdSpeed = fwdSpeed * 2;
        }
        if (gamepad1.a) { //crazy mode
            ySpeed = gamepad1.left_stick_x;
            xSpeed = gamepad1.left_stick_y;
        }
        telemetry.addData("forward Speed", fwdSpeed);
        telemetry.addData("X speed", xSpeed);
    }
}