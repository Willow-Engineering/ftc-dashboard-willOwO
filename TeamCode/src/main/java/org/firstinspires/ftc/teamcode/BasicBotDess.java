//
//
///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//
//import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ServoPulseWidthParameters;
//
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp(name="Basic_bot_DH", group="Linear Opmode")
//@Config
////@Disabled
//public class BasicBotDess extends LinearOpMode {
//
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//    private DcMotorEx arm = null;
//    private Servo claw1 = null;
//    private Servo claw2 = null;
//    private TouchSensor limit;
//
//
//    //
////    claw position variables.
//    public static int left_claw_open = 0;
//    public static int right_claw_open = 50;
//    public static int left_claw_shut = 50;
//    public static int right_claw_shut = 0;
//
//
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//        arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
//        claw1 = hardwareMap.get(Servo.class, "claw1");
//        claw2 = hardwareMap.get(Servo.class, "claw2");
//        limit = hardwareMap.get(TouchSensor.class, "Limit");
//
//
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            if(gamepad1.a == true) {
//                leftPower = Range.clip(drive + turn, -1.5, 1.5);
//                rightPower = Range.clip(drive - turn, -1.5, 1.5);
//            }
//            else {
//                leftPower = Range.clip(drive + turn, -1.0, 1.0);
//                rightPower = Range.clip(drive - turn, -1.0, 1.0);
//            }
//            leftDrive.setPower(leftPower);
//            rightDrive.setPower(rightPower);
//            //base touch sensor code, turns motor off if sensor is triggered
////            if (touch.getState()){
////                //Touch Sensor is not pressed
////                arm.setPower(0.2);
////
////            } else {
////                //Touch Sensor is pressed
////                arm.setPower(0);
//            if (limit.isPressed()) {
//                arm.setTargetPosition(10);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(1500);
//            }
////            if (limit.isPressed()) {
////                rightDrive.setPower(0);
////            }
////            if(limit.isPressed()) {
////                leftDrive.setPower(0);
////            }
////            if(limit.isPressed()) {
////                claw1.setPosition(left_claw_shut);
////            }
////            if(limit.isPressed()) {
////                claw2.setPosition(right_claw_shut);
////            }
//
//
//
//// Up and down on Dpad arm control (basic controls) [DISABLED]
////            if(gamepad1.dpad_up){
////                arm.setPower(0.8);
////            }
////            else if (gamepad1.dpad_down){
////                arm.setPower(-0.8);
////            }
////            else {
////                arm.setPower(0.05);
////            }
//
//            //Set position arm code
//            if (gamepad1.b) {
//                arm.setTargetPosition(0);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(1500);
//            }
////            if (gamepad1.a) {
////                arm.setTargetPosition(1900);
////                arm.setMode(DcMotor.RunMode.RUN_TO_POSI
////                TION);
////                arm.setVelocity(1500);
////            }
//            if (gamepad1.dpad_up) {
//                arm.setTargetPosition(arm.getCurrentPosition() + 80);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(1500);
//            }
//            if (gamepad1.dpad_down) {
//                arm.setTargetPosition(arm.getCurrentPosition() - 80);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(1500);
//            }
////                // Start the motor moving by setting the max velocity to 200 ticks per second
////                arm.setVelocity(2000);
//////                if (gamepad1.x) {
//////                    arm.setTargetPosition(650);
//////                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//////                    arm.setVelocity(1500);
//////                }
////                arm.setVelocity(1500);
//////                if (gamepad1.a) {
//////                    arm.setTargetPosition(1750);
//////                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//////                    arm.setVelocity(1500);
//////                }
//
//            //grabber controls
//            if (gamepad1.right_bumper == true) {
//                claw1.setPosition(left_claw_open);
//
//            }
////                else {
////                    claw1.setPosition(left_claw_shut);
////                }
//            else if (gamepad1.left_bumper ==true) {
//                claw1.setPosition(left_claw_shut);
//            }
//            if (gamepad1.right_bumper == true) {
//                claw2.setPosition(right_claw_open);
//            }
//            else if (gamepad1.left_bumper ==true) {
//                claw2.setPosition(right_claw_shut);
//            }
////                 else {
////                        claw2.setPosition(right_claw_shut);
////                    }
//
//
//
////            telemetry.addData("Arm Test", arm.getCurrentPosition());
////            telemetry.update();
//            // Tank Mode uses one stick to control each wheel. [DISABLED]
//            // - This requires no math, but it is hard to drive forward slowly and keep straight.
//            // leftPower  = -gamepad1.left_stick_y ;
//            // rightPower = -gamepad1.right_stick_y ;
//
//            // Send calculated power to wheels
//
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.addData("Arm_Position", arm.getCurrentPosition());
//            telemetry.addData("Pressed", limit.isPressed());
//            telemetry.addData("claw1", claw1.getPosition());
//            telemetry.addData("claw2", claw2.getPosition());
//            telemetry.update();
//        }
//    }
//}
//
//
