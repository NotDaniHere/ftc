package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "axis_dani")
@Config
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
    public static float position = 1;
    public static boolean running = true;

    // Movement variables
    boolean backward = false;
    double newTarget;

    int currClawPos=0; //0-open 1-closed


    @Override
    public void init() {
        clawGrip = hardwareMap.get(Servo.class, "clawgrip");
        armTilt = hardwareMap.get(Servo.class, "armtilt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor4");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

         // Start with claw closed
        armTilt.setPosition(0.1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {
        //comment after testing
        //clawGrip.setPosition(position);
        // Handle movement controls
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;

        // Handle sliding controls
        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
        slide(slidePower);

        // Handle backward toggle with edge detection
        rightBumperPrev = gamepad1.right_bumper;


        // Arm vertical movement (Y/B buttons)
        if (gamepad2.y) {
            armMotor.setPower(-1); // Move arm up
        } else if (gamepad2.b) {
            armMotor.setPower(1); // Move arm down
        } else {
            armMotor.setPower(0);
        }
        //0.4 apasa tare, 0.1 open pentru cubik
        if(gamepad2.a) {
            clawGrip.setPosition(0.4);
        } else {
            clawGrip.setPosition(0.1);
        }


        // Claw toggle (A button)
        aPrev = gamepad1.a;

        // Smooth arm tilting with right stick
        //double targetTilt = (-gamepad1.right_stick_y * 0.5 + 0.5); // Convert -1->1 to 0->1
        //targetTilt = Math.max(MIN_POS, Math.min(targetTilt, MAX_POS)); // Constrain to limits
        //currentTiltPosition += (targetTilt - currentTiltPosition) * SMOOTHING_FACTOR;
        //armTilt.setPosition(currentTiltPosition);he arm.
        if(gamepad2.x) {armTilt.setPosition(0.5);} else {armTilt.setPosition(0.1);}
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
        double frontRightPower = y + x;
        double frontLeftPower = y - x;
        double backRightPower = y + x;
        double backLeftPower = y - x;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }
    private void slide(double slidePower) {
        double frontLeftPower = -slidePower;
        double frontRightPower = slidePower;
        double backLeftPower = slidePower;
        double backRightPower = -slidePower;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}