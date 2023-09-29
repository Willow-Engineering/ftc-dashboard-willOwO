package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp()
public class gamepadredo extends OpMode {
    private DcMotor motor1;
    private Servo servo1;
    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        servo1 = hardwareMap.get(Servo.class, "servo1");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
    }
    @Override
    public void loop() {
        //motor control code, runs motor 1 when pressing B
        if (gamepad1.b) {
            motor1.setPower(1);
        }
        if (gamepad1.y) {
            servo1.setPosition(80);
        }
    }
}
