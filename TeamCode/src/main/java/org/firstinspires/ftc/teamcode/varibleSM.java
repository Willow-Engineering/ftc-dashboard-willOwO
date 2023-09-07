package org.firstinspires.ftc.teamcode;

         import com.acmerobotics.dashboard.FtcDashboard;
         import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
         import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
         @TeleOp()
 public class varibleSM extends OpMode {
 /**
  * This is called when the driver presses INIT
  */
         @Override
 public void init() {

             int teamNumber = 16072;
             double motorSpeed = 0.5;
             boolean touchSensorPressed = true;

         // this sends to the driver station
             telemetry.addData("Team Number", teamNumber);
             telemetry.addData("Motor Speed", motorSpeed);
             telemetry.addData("Touch Sensor", touchSensorPressed);

             FtcDashboard dashboard = FtcDashboard.getInstance();
             telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

         }

         /**
  * This is called repeatedly while OpMode is playing
  */@Override
public void loop() {
         // intentionally left blank
         }
}