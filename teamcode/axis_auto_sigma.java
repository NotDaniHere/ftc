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

@Autonomous(name="axis_auto_sigma")
@Config
public class axis_auto_sigma extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Configurable parameters
    public static double ROTATE_POWER = 0.6;
    public static double DRIVE_POWER = -0.5;
    public static double ANGLE_TOLERANCE = 2.0;
    public static int X_MS = 600;
    public static int Y_MS = 1500;
    public static int Z_MS = 700;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();

        // Execute movement sequence
        executeMovementSequence();

        telemetry.addData("Status", "Program Complete!");
        telemetry.update();
    }

    private void initializeHardware() {
        // Motor initialization
        frontLeft = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        backLeft = hardwareMap.get(DcMotor.class, "motor3");
        backRight = hardwareMap.get(DcMotor.class, "motor4");

        // Reverse right motors for mecanum
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    private void executeMovementSequence() throws InterruptedException {
        // Sequence: 90° → X ms → 180° → X ms → 90° → Y ms → 270° → Z ms
        rotateToAngle(90);
        moveForwardTime(DRIVE_POWER, X_MS);

        rotateToAngle(180);
        moveForwardTime(DRIVE_POWER, X_MS);

        rotateToAngle(90);
        moveForwardTime(DRIVE_POWER, Y_MS);

        rotateToAngle(270);
        moveForwardTime(DRIVE_POWER, Z_MS);
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

            setMotorPowers(power, -power);
            telemetry.addData("Rotating", "Target: %.1f° Current: %.1f°", targetAngle, currentHeading);
            telemetry.update();
        }
        stopMotors();
    }

    private void moveForwardTime(double power, int durationMs) {
        setMotorPowers(power, power);
        sleep(durationMs);
        stopMotors();
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    private void stopMotors() {
        setMotorPowers(0, 0);
    }
}