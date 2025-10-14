package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * A simple differential (tank) drive helper class.
 * Supports both tank and arcade drive modes with normalization,
 * deadband, exponent shaping, and optional motor inversion.

 * Inspired by WPILib’s DifferentialDrive, adapted for FTC SDK.
 */
public class DifferentialDrive {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private double maxSpeed = 1.0;

    private double slowModeCoefficient = 0.25;
    private double deadband = 0.01;
    private double exponent = 1.0;


    private boolean slowModeEnabled = false;

    private boolean breakModeEnabled = true;

    public DifferentialDrive(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        setBrakeMode(breakModeEnabled, true);
        setSlowModeEnabled(slowModeEnabled);
    }

    // === DRIVE METHODS ===

    /**
     * Tank drive: each stick controls one side.
     */
    public void tankDrive(double leftInput, double rightInput) {
        leftInput = applyDeadband(leftInput);
        rightInput = applyDeadband(rightInput);

        leftInput = shape(leftInput);
        rightInput = shape(rightInput);

        // Apply inversion
        double leftPower = Range.clip(leftInput, -1, 1);
        double rightPower = Range.clip(rightInput, -1, 1);

        // Normalize (make sure max magnitude is 1)
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        drive(leftPower, rightPower);
    }

    /**
     * Arcade drive: one stick controls forward/back, the other controls turn.
     */
    public void arcadeDrive(double drive, double turn) {
        drive = applyDeadband(drive);
        turn = applyDeadband(turn);

        drive = shape(drive);
        turn = shape(turn);

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        drive(leftPower, rightPower);
    }

    public void stop() {
        drive(0, 0);
    }

    private void drive(double left, double right) {
        double coefficient = maxSpeed * (slowModeEnabled ? slowModeCoefficient : 1);
        leftMotor.setPower(left * coefficient);
        rightMotor.setPower(right * coefficient);
    }

    // === SETTERS ===

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public void setExponent(double exponent) {
        this.exponent = exponent;
    }

    public void setSlowModeEnabled(boolean slowModeEnabled) {
        this.slowModeEnabled = slowModeEnabled;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void setSlowModeCoefficient(double slowModeCoefficient) {
        this.slowModeCoefficient = slowModeCoefficient;
    }

    // === MOTOR CONFIG ===

    private void setBrakeMode(boolean enabled, boolean force) {
        if (breakModeEnabled != enabled || force) {
            DcMotor.ZeroPowerBehavior mode = enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

            leftMotor.setZeroPowerBehavior(mode);
            rightMotor.setZeroPowerBehavior(mode);

            breakModeEnabled = enabled;
        }
    }

    public void setBrakeMode(boolean enabled) {
        setBrakeMode(enabled, false);
    }

    public void setInverted(boolean leftInverted, boolean rightInverted) {
        leftMotor.setDirection(leftInverted ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(rightInverted ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    // === GETTERS ===

    public double getLeftPower() {
        return leftMotor.getPower();
    }

    public double getRightPower() {
        return rightMotor.getPower();
    }

    public boolean isBreakModeEnabled() {
        return breakModeEnabled;
    }

    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

    // === UTIL ===

    private double applyDeadband(double value) {
        double absValue = Math.abs(value);
        if (absValue < deadband) return 0;
        return Math.copySign((absValue - deadband) / (1 - deadband), value);
    }


    private double shape(double value) {
        double absValue = Math.abs(value);
        return Math.copySign(Math.pow(absValue, exponent), value);
    }
}
