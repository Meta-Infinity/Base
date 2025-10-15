package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Direction Test", group = "Test")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Stop all motors initially
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addLine("Ready to test motors. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Press gamepad buttons to test each motor individually
            if (gamepad1.a) {
                frontLeftMotor.setPower(0.5);
                telemetry.addLine("Testing FRONT LEFT motor");
            } else if (gamepad1.b) {
                frontRightMotor.setPower(0.5);
                telemetry.addLine("Testing FRONT RIGHT motor");
            } else if (gamepad1.x) {
                backLeftMotor.setPower(0.5);
                telemetry.addLine("Testing BACK LEFT motor");
            } else if (gamepad1.y) {
                backRightMotor.setPower(0.5);
                telemetry.addLine("Testing BACK RIGHT motor");
            } else {
                // Stop all if no button pressed
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            telemetry.update();
        }
    }
}
