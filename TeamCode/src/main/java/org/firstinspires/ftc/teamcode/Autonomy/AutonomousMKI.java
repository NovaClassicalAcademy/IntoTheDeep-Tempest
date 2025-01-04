package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Rei Auto Run MKI", group = "Rei")
public class AutonomousMKI extends LinearOpMode {
    HardwareRobot _robot = new HardwareRobot();

    final double _driveSpeed = 0.05;
    final double _turnSpeed = 0.05;
    final double _liftPower = 0.25;

    @Override
    public void runOpMode() {
        _robot.Init(hardwareMap);

        telemetry.addData("Initialization", "Started");
        InitializeLiftPosition();

        telemetry.addData("Initialization", "Complete");
        telemetry.addData("Front Left Drive", _robot.FrontLeftDrive.getCurrentPosition());
        telemetry.addData("Front Right Drive", _robot.FrontRightDrive.getCurrentPosition());
        telemetry.addData("Back Left Drive", _robot.BackLeftDrive.getCurrentPosition());
        telemetry.addData("Back Right Drive", _robot.BackRightDrive.getCurrentPosition());
        telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();

        MoveForwardBackWards(12, 6);
        sleep(2000);
        MoveForwardBackWards(-12, 6);
        sleep(2000);

        Turn(45, 6);
        sleep(2000);
        Turn(-45, 6);
        sleep(2000);
        SetLiftPosition(1500, 5);
        sleep(2000);
        Dump();
        sleep(2000);
        RetractDump();
        sleep(2000);
        SetLiftPosition(0, 5);
        sleep(2000);
        LowerHinge();
        sleep(2000);
        OpenClaw();
        sleep(2000);
        CloseClaw();
        sleep(2000);
        LiftHinge();
        sleep(2000);
        OpenGripper();
        sleep(2000);
        CloseGripper();

    }

    private void MoveForwardBackWards(double moveInches, double timeOut) {
        if (!opModeIsActive()) {
            return;
        }

        _robot.YEncoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _robot.YEncoderWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int moveTicks = ConvertInchesToTicks(moveInches);
        double currentPosition = _robot.YEncoderWheel.getCurrentPosition();
        double targetPosition = currentPosition + moveTicks;

        double startTime = getRuntime();
        double runTime;

        if (moveInches < 0) {
            _robot.FrontLeftDrive.setPower(_driveSpeed);
            _robot.FrontRightDrive.setPower(_driveSpeed);
            _robot.BackLeftDrive.setPower(_driveSpeed);
            _robot.BackRightDrive.setPower(_driveSpeed);

            while (currentPosition > targetPosition) {

                runTime = getRuntime() - startTime;
                currentPosition = _robot.YEncoderWheel.getCurrentPosition();

                if (runTime > timeOut) { break; }

                telemetry.addData("Time Out Time", timeOut);
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Distance travelled", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        } else if (moveInches > 0) {
            _robot.FrontLeftDrive.setPower(-_driveSpeed);
            _robot.FrontRightDrive.setPower(-_driveSpeed);
            _robot.BackLeftDrive.setPower(-_driveSpeed);
            _robot.BackRightDrive.setPower(-_driveSpeed);

            while (currentPosition < targetPosition) {

                runTime = getRuntime() - startTime;
                currentPosition = _robot.YEncoderWheel.getCurrentPosition();

                if (runTime > timeOut) { break; }

                telemetry.addData("Time Out Time", timeOut);
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Distance travelled", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        }

        _robot.FrontLeftDrive.setPower(0.0);
        _robot.FrontRightDrive.setPower(0.0);
        _robot.BackLeftDrive.setPower(0.0);
        _robot.BackRightDrive.setPower(0.0);
    }

    private void SetLiftPosition(double newPosition, double timeOut) {
        if (!opModeIsActive()) {
            return;
        }

        double currPosition = _robot.LeftLiftMotor.getCurrentPosition();
        double targetPosition = currPosition + newPosition;

        // Set min/max limits for the lift.
        if (targetPosition > 2900)
        { targetPosition = 2900; }

        if (targetPosition < 0)
        { targetPosition = 0; }

        double startTime = getRuntime();
        double runTime;

        if (currPosition > targetPosition) {
            _robot.LeftLiftMotor.setPower(_liftPower);
            _robot.RightLiftMotor.setPower(_liftPower);

            while (currPosition > targetPosition) {
                currPosition = _robot.LeftLiftMotor.getCurrentPosition();
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) { break; }

                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.update();
            }
        } else if (currPosition < targetPosition) {
            _robot.LeftLiftMotor.setPower(-_liftPower);
            _robot.RightLiftMotor.setPower(-_liftPower);

            while (currPosition < targetPosition) {
                currPosition = _robot.LeftLiftMotor.getCurrentPosition();
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) { break; }

                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.update();
            }
        }

        if (currPosition > 1500) {
            // If lift is extended more than halfway,
            // give the lift a tiny amount of power
            // to help hold the position.
            _robot.LeftLiftMotor.setPower(0.005);
            _robot.RightLiftMotor.setPower(0.005);
        }
        else {
            _robot.LeftLiftMotor.setPower(0.0);
            _robot.RightLiftMotor.setPower(0.0);
        }
    }

    private void Turn(double degreeChange, double timeout) {
        if (!opModeIsActive()) {
            return;
        }

        _robot.TurnGyro.resetYaw();

        double currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
        double targetAngle = currAngle + degreeChange;
        double startTime = getRuntime();
        double runTime;

        if (degreeChange < 0) {
            _robot.FrontLeftDrive.setPower(_turnSpeed);
            _robot.FrontRightDrive.setPower(-_turnSpeed);
            _robot.BackLeftDrive.setPower(_turnSpeed);
            _robot.BackRightDrive.setPower(-_turnSpeed);

            while (currAngle > targetAngle) {
                currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
                runTime = getRuntime() - startTime;

                if (runTime > timeout) { break; }

                telemetry.addData("Angle", currAngle);
                telemetry.addData("Target Angle", degreeChange);
                telemetry.update();
            }
        } else if (degreeChange > 0) {
            _robot.FrontLeftDrive.setPower(-_turnSpeed);
            _robot.FrontRightDrive.setPower(_turnSpeed);
            _robot.BackLeftDrive.setPower(-_turnSpeed);
            _robot.BackRightDrive.setPower(_turnSpeed);

            while (currAngle < targetAngle) {
                currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
                runTime = getRuntime() - startTime;

                if (runTime > timeout) { break; }

                telemetry.addData("Angle", currAngle);
                telemetry.addData("Target Angle", degreeChange);
                telemetry.update();
            }
        }

        _robot.FrontLeftDrive.setPower(0.0);
        _robot.FrontRightDrive.setPower(0.0);
        _robot.BackLeftDrive.setPower(0.0);
        _robot.BackRightDrive.setPower(0.0);
    }

    private void Dump() {
        double endPosition = 0.25;

        _robot.DumpBucketServo.setPosition(endPosition);

        while (_robot.DumpBucketServo.getPosition() != endPosition) {

            telemetry.addData("Dump Rotation", _robot.DumpBucketServo.getPosition());
            telemetry.update();
        }
    }

    private void RetractDump() {
        double endPosition = 1;

        _robot.DumpBucketServo.setPosition(endPosition);

        while (_robot.DumpBucketServo.getPosition() != endPosition) {

            telemetry.addData("Dump Rotation", _robot.DumpBucketServo.getPosition());
            telemetry.update();
        }
    }

    private void OpenClaw() {
        _robot.LeftClawServo.setPosition(0.0);
        _robot.RightClawServo.setPosition(1.0);

        while (_robot.LeftClawServo.getPosition() != 0.0 && _robot.RightClawServo.getPosition() != 1.0) {

            telemetry.addData("Left Claw Rotation", _robot.LeftClawServo.getPosition());
            telemetry.addData("Right Claw Rotation", _robot.RightClawServo.getPosition());
            telemetry.update();
        }
    }

    private void CloseClaw() {
        _robot.LeftClawServo.setPosition(0.44);
        _robot.RightClawServo.setPosition(0.56);

        while (_robot.LeftClawServo.getPosition() != 0.44 && _robot.RightClawServo.getPosition() != 0.56) {

            telemetry.addData("Left Claw Rotation", _robot.LeftClawServo.getPosition());
            telemetry.addData("Right Claw Rotation", _robot.RightClawServo.getPosition());
            telemetry.update();
        }
    }

    private void OpenGripper() {
        _robot.GripperServo.setPosition(0.15);

        while (_robot.GripperServo.getPosition() != 0.15) {

            telemetry.addData("Grip Rotation", _robot.GripperServo.getPosition());
            telemetry.update();
        }
    }

    private void CloseGripper() {
        _robot.GripperServo.setPosition(0.8);

        while (_robot.GripperServo.getPosition() != 0.8) {

            telemetry.addData("Grip Rotation", _robot.GripperServo.getPosition());
            telemetry.update();
        }
    }

    private void LiftHinge() {
        double currentHingePosition = _robot.LeftHingeServo.getPosition();

        _robot.LeftHingeServo.setPosition(1.0);
        _robot.RightHingeServo.setPosition(0.25);

        while (currentHingePosition != 1.0) {
            currentHingePosition = _robot.LeftHingeServo.getPosition();

            telemetry.addData("Hinge Rotation", currentHingePosition);
            telemetry.update();
        }
    }

    private void LowerHinge() {
        double currentHingePosition = _robot.LeftHingeServo.getPosition();

        _robot.LeftHingeServo.setPosition(0.675);
        _robot.RightHingeServo.setPosition(0.575);

        while (currentHingePosition != 0.675) {
            currentHingePosition = _robot.LeftHingeServo.getPosition();

            telemetry.addData("Hinge Rotation", currentHingePosition);
            telemetry.update();
        }
    }

    private  int ConvertInchesToTicks(double inches) {
        int encoderTicksPerRotation = 2000;
        double encoderDiameter = 1.82;

        double circumference = encoderDiameter * Math.PI;
        double rotations = 1 / circumference;
        int ticks = (int)Math.floor(rotations * inches * encoderTicksPerRotation);
        return  ticks;
    }

    private void InitializeLiftPosition() {
        _robot.LeftLiftMotor.setPower(0.05);
        _robot.RightLiftMotor.setPower(0.05);

        sleep(1000);

        _robot.LeftLiftMotor.setPower(0);
        _robot.RightLiftMotor.setPower(0);

        sleep(1000);

        _robot.LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _robot.RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _robot.LeftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _robot.RightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
