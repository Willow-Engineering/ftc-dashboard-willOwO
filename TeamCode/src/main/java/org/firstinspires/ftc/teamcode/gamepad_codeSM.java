//
//
//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//
//
//@TeleOp()
//public class gamepad_codeSM extends OpMode {
//    @Override
//    private DcMotor motor1;
//
//    public void init() {
//    }
//    testMotor1 = hardwareMap.get(DcMotor.class, "testMotor1");
//    testMotor2 = hardwareMap.get(DcMotor.class, "testMotor2");
//
//
//    if(gamepad1.b) {
//        testMotor1.setPower(1);
//    }
//
//
//    @Override
//    public void loop() {
//        //left stick controls and buttons A+B:
//        double speedForward = -gamepad1.left_stick_y / 2.0;
//        telemetry.addData("Left stick x", gamepad1.left_stick_x);
//        telemetry.addData("Left stick y", gamepad1.left_stick_y);
//        telemetry.addData("A button", gamepad1.a);
//        telemetry.addData("B button", gamepad1.b);
//        telemetry.addData("Right trigger", gamepad1.right_trigger);
//        telemetry.addData("Left trigger", gamepad1.left_trigger);
//        telemetry.addData("speed Forward", speedForward);
//        //right stick controls:
//        telemetry.addData("Right stick x", gamepad1.right_stick_x);
//        telemetry.addData("Right stick y", gamepad1.right_stick_y);
//        telemetry.addData("Trigger sum", gamepad1.right_trigger + gamepad1.left_trigger);
//        if (gamepad1.left_stick_y < -0.5) {
//            telemetry.addData("Left stick", "is negative and large");
//        }
//    }
//}