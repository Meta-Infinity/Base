package org.firstinspires.ftc.teamcode.Subsystems;

//import static org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive.PARAMS;

import androidx.annotation.NonNull;

//import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private static DcMotorEx leftFront, leftBack, rightFront, rightBack;
    //    private static SparkFunOTOSCorrected otos;
    private static SparkFunOTOS.Pose2D pose;
    private static double turnNerf = .5;
    private static double driveNerf = 1;

    public static double lastFrontLeft = 0;
    public static double lastFrontRight = 0;
    public static double lastBackLeft = 0;
    public static double lastBackRight = 0;
    private Drive() { }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) { }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftBack = hwmap.get(DcMotorEx.class, "backLeftMotor");
        leftFront = hwmap.get(DcMotorEx.class, "frontLeftMotor");
        rightBack = hwmap.get(DcMotorEx.class, "backRightMotor");
        rightFront = hwmap.get(DcMotorEx.class, "frontRightMotor");
//        odo = hwmap.get(GoBildaPinpointDriver.class,"pinpoint");
//        odo.setOffsets(-84.0, -168.0);
//        odo.setEncoderResolution(4096.0/(35.0*Math.PI));
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();

//        otos = hwmap.get(SparkFunOTOSCorrected.class, "otos");
//        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
//        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));
//        otos.setAngularUnit(AngleUnit.RADIANS);
//        System.out.println(otos.calibrateImu(255, false));
        setDefaultCommand(driveCommand());
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode){ //if (otos.getImuCalibrationProgress() == 0){ setDefaultCommand(driveCommand()); }
    }
    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode){ //if (otos.getImuCalibrationProgress() == 0){ setDefaultCommand(driveCommand()); }
    }

    public static void driveUpdate() {
//        odo.update();
//        Mercurial.gamepad1().dpadLeft().onTrue(resetIMU());
        // read the gamepads
        double rightX = Mercurial.gamepad1().leftStickX().state();
        double rightY = Mercurial.gamepad1().leftStickY().state();
        double turn = Mercurial.gamepad1().rightStickX().state();
        if (Mercurial.gamepad1().rightTrigger().state() > 0.05) {
            turn *= 0.75;
        }
        //turn*=0.75;
//        Mercurial.gamepad1().rightBumper().onTrue(toggleDriveNerf());
//        Pose2D pos = odo.getPosition();

//        double heading = 0; //otos.getPosition().h;
        double heading = 0;//pos.getHeading(AngleUnit.RADIANS);
        // Do the kinematics math
        double rotX = (rightX * Math.cos(-heading) - rightY * Math.sin(-heading));
        double rotY = rightX * Math.sin(-heading) + rightY * Math.cos(-heading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double lfPower = (rotY + rotX + turn) / denominator;
        double lbPower = (rotY - rotX + turn) / denominator;
        double rfPower = (rotY - rotX - turn) / denominator;
        double rbPower = (rotY + rotX - turn) / denominator;
        // Set the Motor Power
        leftBack.setPower(lbPower);
        leftFront.setPower(lfPower);
        rightBack.setPower(rbPower);
        rightFront.setPower(rfPower);

        lastBackLeft = lbPower;
        lastFrontLeft = lfPower;
        lastBackRight = rbPower;
        lastFrontRight = rfPower;
    }

    private static void resetHeading(){
//        otos.setPosition(IMU);
//        odo.resetPosAndIMU();
    }

    public static double getHeading() {
        return 0;
                //odo.getPosition().getHeading();
    }

    @NonNull
    private static Lambda toggleDriveNerf() {
        return new Lambda("nerf drive speed")
                .setExecute(() -> {
                    if(driveNerf==1) {
                        driveNerf = .5;
                    } else {
                        driveNerf = 1;
                    }
        });
    }

    @NonNull
    private static Lambda resetIMU() {
        return new Lambda("reset imu")
                .setExecute(() -> {
                    //odo.resetPosAndIMU();
                });
    }

    @NonNull
    public static Lambda nerfDrive(double multiplier){
        return new Lambda("nerf turn speed")
                .setExecute(() -> {turnNerf = multiplier; });
    }


    @NonNull
    public static Lambda driveCommand() {
        return new Lambda("driveCommand")
                .addRequirements(INSTANCE)
                .setExecute(Drive::driveUpdate)
                .setInterruptible(() -> true)
                .setFinish(()->false);
    }

    @NonNull
    public static Lambda zeroHeading() {
        return new Lambda("zeroHeading")
                .addRequirements(INSTANCE)
                .setInit(Drive::resetHeading);
    }
}