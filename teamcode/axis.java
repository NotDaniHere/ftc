package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "axis_dani")
public class axis extends OpMode {
    public Servo clawGrip;
    public Servo armTilt; // This should be Servo type
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;

    // Add missing constants
    private static final double MIN_POS = 0.15;
    private static final double MAX_POS = 0.85;
    private static final double SMOOTHING_FACTOR = 0.1;

    // Servo control variables
    private boolean clawOpen = false;
    private boolean aPrev = false;
    private boolean rightBumperPrev = false;
    private double currentTiltPosition = 0.5;

    // Movement variables
    boolean backward = false;
    double newTarget;

    @Override
    public void init() {
        clawGrip = hardwareMap.get(Servo.class, "clawgrip");
        armTilt = hardwareMap.get(Servo.class, "armtilt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");

        clawGrip.setPosition(0.5); // Start with claw closed
        armTilt.setPosition(currentTiltPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle movement controls
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;

        // Handle sliding controls
        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Handle backward toggle with edge detection
        rightBumperPrev = gamepad1.right_bumper;

        // Arm vertical movement (Y/B buttons)
        if (gamepad1.y) {
            armMotor.setPower(1); // Move arm up
        } else if (gamepad1.b) {
            armMotor.setPower(-1); // Move arm down
        } else {
            armMotor.setPower(0);
        }

        // Claw toggle (A button)
        if (gamepad1.a && !aPrev) {
            clawOpen = !clawOpen;
            clawGrip.setPosition(clawOpen ? 0.7 : 0.3);
        }
        aPrev = gamepad1.a;

        // Smooth arm tilting with right stick
        double targetTilt = (-gamepad1.right_stick_y * 0.5 + 0.5); // Convert -1->1 to 0->1
        targetTilt = Math.max(MIN_POS, Math.min(targetTilt, MAX_POS)); // Constrain to limits
        currentTiltPosition += (targetTilt - currentTiltPosition) * SMOOTHING_FACTOR;
        armTilt.setPosition(currentTiltPosition);

        // Apply movement with backward toggle
        deplasare(x,y);

        // Update telemetry
        telemetry.addData("Backward Mode", backward);
        telemetry.addData("Claw State", clawOpen ? "Open" : "Closed");
        telemetry.addData("Arm Tilt Position", "%.2f", currentTiltPosition);
        telemetry.addData("Slide Power", "%.2f", slidePower);
        telemetry.update();
    }

    private void deplasare(float x, float y) {
        double frontLeftPower = y + x;
        double frontRightPower = y - x;
        double backLeftPower = y + x;
        double backRightPower = y - x;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }
}