package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="tuner")
@Config

public class ServoTester extends LinearOpMode {
    public static String name = "";
    public static String name2 = "";
    boolean isGrabbing = false, changed = false;
    public static double pos1, pos2;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo = hardwareMap.servo.get(name);
        Servo servo2 = hardwareMap.servo.get(name2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.x) {
                if (!changed) {
                    isGrabbing = !isGrabbing;
                    changed = true;
                }
            } else {
                changed = false;
            }
            servo.setPosition(isGrabbing ? (double) pos1 : (double) pos2);
            servo2.setPosition(isGrabbing ? (double) (pos1) : (double) (pos2));
//            servo.setPosition(pos1);
//            servo2.setPosition(pos2);
            telemetry.addData("position", servo.getPosition());
            telemetry.addData("position2", servo2.getPosition());
            telemetry.addData("pos", isGrabbing ? 1 : 2);
            telemetry.update();
        }
    }
}