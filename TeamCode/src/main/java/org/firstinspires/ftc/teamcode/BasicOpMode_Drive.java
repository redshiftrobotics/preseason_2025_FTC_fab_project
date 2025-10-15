package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive POV (Default)", group = "MichaelL")
public class BasicOpMode_Drive extends OpMode {
    private DifferentialDrive drive;
    private final ElapsedTime runtime = new ElapsedTime();

    private boolean drivePOV = true;

    private DcMotor device1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");


        // Drive Setup

        DcMotor left = hardwareMap.get(DcMotor.class, "l");
        DcMotor right = hardwareMap.get(DcMotor.class, "r");

        drive = new DifferentialDrive(left, right);
        drive.setInverted(true, false);
        drive.setDeadband(0.05);
        drive.setExponent(1.2);
        drive.setSlowModeCoefficient(0.3);

        // Device Setup

        device1 = hardwareMap.get(DcMotor.class, "d");
        device1.setDirection(DcMotorSimple.Direction.FORWARD);
        device1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
        gamepad1.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime);

        // --- Drive ---

        telemetry.addLine();
        telemetry.addLine(" - Drive - ");

        if (gamepad1.bWasPressed()) {
            drive.setSlowModeEnabled(!drive.isSlowModeEnabled());
            gamepad1.rumble(0, 0.3, 150);
        }

        drive.setBrakeMode(!gamepad1.left_bumper);

        telemetry.addData("Modes", "speed %s, drift %s", drive.isSlowModeEnabled() ? "slow" : "fast", drive.isBreakModeEnabled() ? "break" : "coast");

        if (drivePOV) {
            double drivePower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;
            drive.arcadeDrive(drivePower, turnPower);
            telemetry.addData("Joystick", "drive %.2f, turn %.2f", drivePower, turnPower);
        } else {
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;
            drive.tankDrive(leftPower, rightPower);
            telemetry.addData("Joystick", "left %.2f, right %.2f", leftPower, rightPower);
        }

        telemetry.addData("Motors", "left %.2f, right %.2f", drive.getLeftPower(), drive.getRightPower());

        // --- LED ---

        if (!drive.isBreakModeEnabled()) {
            gamepad1.setLedColor(1, 0, 0, -1);
        } else if (Math.abs(drive.getLeftPower()) + Math.abs(drive.getRightPower()) > 0) {
            gamepad1.setLedColor(1, 1, 1, -1);
        } else if (drive.isSlowModeEnabled()) {
            gamepad1.setLedColor(0, 1, 0, -1);
        } else {
            gamepad1.setLedColor(0, 0, 0, -1);
        }

        // --- Mechanism ---

        telemetry.addLine();
        telemetry.addLine(" - Mechanism - ");

        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(power) < 0.1) power = 0;
        device1.setPower(power);
        telemetry.addData("Device1 Power", device1.getPower());
        telemetry.addData("Device1 Position", device1.getCurrentPosition());
    }

    @Override
    public void stop() {
        drive.stop();
        telemetry.addData("Status", "Stopped");
        gamepad1.setLedColor(0, 0, 0, -1);
    }

    public void setDrivePOV(boolean drivePOV) {
        this.drivePOV = drivePOV;
    }
}
