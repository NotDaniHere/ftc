package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="axis_auto")
@Config
public class axis_auto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, armMotor;
    private IMU imu;

    // Configurable parameters (adjust via FTC Dashboard)
    public static double DRIVE_POWER = -0.5;
    public static double ROTATE_POWER = 0.6;
    public static double ARM_POWER = 1.0;
    public static int X_MS = 1000;
    public static int Y_MS = 1000;
    public static int Z_MS = 1000;
    public static double ANGLE_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();

        // Full movement sequence
        executeAutonomousSequence();

        telemetry.addData("Status", "Program Complete!");
        telemetry.update();
    }

    private void initializeHardware() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        backLeft = hardwareMap.get(DcMotor.class, "motor3");
        backRight = hardwareMap.get(DcMotor.class, "motor4");

        // Reverse right motors for mecanum
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Arm motor
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    private void executeAutonomousSequence() throws InterruptedException {
        // First rotation and movement
        rotateToAngle(90);
        driveForward(X_MS);
        operateArm(ARM_POWER, 5000);    // Raise arm for 5s
        operateArm(-ARM_POWER, 2000);  // Lower arm for 2s

        // Second rotation and movement
        rotateToAngle(180);
        driveForward(X_MS);

        // Third rotation and movement
        rotateToAngle(90);
        driveForward(Y_MS);

        // Final rotation and arm operation
        rotateToAngle(270);
        operateArm(ARM_POWER, 5000);   // Raise arm again for 5s
        driveForward(Z_MS);
    }

    private void rotateToAngle(double targetAngle) throws InterruptedException {
        imu.resetYaw();
        while (opModeIsActive()) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double currentHeading = angles.getYaw(AngleUnit.DEGREES);
            double remaining = AngleUnit.normalizeDegrees(targetAngle - currentHeading);

            if (Math.abs(remaining) <= ANGLE_TOLERANCE) break;

            double power = ROTATE_POWER * (remaining / 180);
            power = Math.copySign(Math.max(Math.abs(power), 0.15), power);

            setDrivePowers(power, -power);
            telemetry.addData("Rotating", "Target: %.1f° Current: %.1f°", targetAngle, currentHeading);
            telemetry.update();
        }
        stopDriveMotors();
    }

    private void driveForward(int durationMs) {
        setDrivePowers(DRIVE_POWER, DRIVE_POWER);
        sleep(durationMs);
        stopDriveMotors();
    }

    private void operateArm(double power, int durationMs) {
        armMotor.setPower(power);
        sleep(durationMs);
        armMotor.setPower(0);
    }

    private void setDrivePowers(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    private void stopDriveMotors() {
        setDrivePowers(0, 0);
    }
}