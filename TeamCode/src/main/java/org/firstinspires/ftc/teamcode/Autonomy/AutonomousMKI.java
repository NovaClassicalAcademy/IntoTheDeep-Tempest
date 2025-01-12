package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Rei Auto Run MKI", group = "Rei")
public class AutonomousMKI extends LinearOpMode {
    HardwareRobot _robot = new HardwareRobot();

    final double _driveSpeed = 0.2;
    final double _turnSpeed = 0.05;
    final double _liftPower = 3.0;

    @Override
    public void runOpMode() {
        _robot.Init(hardwareMap);

        telemetry.addData("Initialization", "Started");
//        InitializeLiftPosition();
//        InitializeHingeAndClawsPosition();
//        InitializeGripper();

        telemetry.addData("Initialization", "Complete");
        telemetry.addData("Front Left Drive", _robot.FrontLeftDrive.getCurrentPosition());
        telemetry.addData("Front Right Drive", _robot.FrontRightDrive.getCurrentPosition());
        telemetry.addData("Back Left Drive", _robot.BackLeftDrive.getCurrentPosition());
        telemetry.addData("Back Right Drive", _robot.BackRightDrive.getCurrentPosition());
        telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();
//        HighBucketRun();
        Park();
    }

    private void Park() {
        MoveForwardBackWards(-5, 3);
    }
    private void HighBucketRun() {
        MoveForwardBackWards(-2, 2);
        SetLiftPosition(2900, 7);
        Dump();
        sleep(1000);
        RetractDump();
        sleep(1000);
        MoveForwardBackWards(2, 2);
        SetLiftPosition(0, 7);
        Turn(-76, 3);
        sleep(1000);
        MoveForwardBackWards(5, 5);
        LowerHinge();
        sleep(1000);
        MoveForwardBackWards(2, 5);
        CloseClaw();
        sleep(1000);
        LiftHinge();
        sleep(1000);
        OpenClaw();
        sleep(1000);
        Turn(51, 3);
        sleep(1000);
        MoveForwardBackWards(-4, 2);
        SetLiftPosition(2900, 7);
        Dump();
        sleep(1000);
        RetractDump();
        sleep(1000);
        MoveForwardBackWards(3, 2);
        SetLiftPosition(0, 7);
        Turn(-68, 4);
        sleep(1000);
        LowerHinge();
        sleep(1000);
        MoveForwardBackWards(3, 1);
        CloseClaw();
        sleep(1000);
        LiftHinge();
        sleep(1000);
        OpenClaw();
        sleep(1000);

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
                telemetry.addData("Move Ticks", moveTicks);
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
                telemetry.addData("Move Ticks", moveTicks);
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

        double currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
        double positionDifference = 2900 - newPosition;
        double targetPosition = newPosition - positionDifference;


        // Set min/max limits for the lift.
        if (targetPosition > 2900) {
            targetPosition = 2900;
        }

        if (targetPosition < 0) {
            targetPosition = 0;
        }

        double startTime = getRuntime();
        double runTime;

        if (currPosition > targetPosition) {
            _robot.LeftLiftMotor.setPower(_liftPower);
            _robot.RightLiftMotor.setPower(_liftPower);

            while (currPosition > targetPosition) {
                currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) {
                    break;
                }

                telemetry.addData("Lift Mode", "Retracting");
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        } else if (currPosition < targetPosition) {
            _robot.LeftLiftMotor.setPower(-_liftPower);
            _robot.RightLiftMotor.setPower(-_liftPower);

            while (currPosition < targetPosition) {
                currPosition = _robot.RightLiftMotor.getCurrentPosition() * -1;
                runTime = getRuntime() - startTime;

                if (runTime > timeOut) {
                    break;
                }

                telemetry.addData("Lift Mode", "Extending");
                telemetry.addData("Elapse Time", runTime);
                telemetry.addData("Time Out", timeOut);
                telemetry.addData("Current Position", currPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
            }
        }

        if (currPosition > 1500) {
            // If lift is extended more than halfway,
            // give the lift a tiny amount of power
            // to help hold the position.
            _robot.LeftLiftMotor.setPower(0.01);
            _robot.RightLiftMotor.setPower(0.01);
        } else {
            _robot.LeftLiftMotor.setPower(0.0);
            _robot.RightLiftMotor.setPower(0.0);
        }

    }

    private void Turn(double targetAngle, double timeout) {
        if (!opModeIsActive()) {
            return;
        }

        _robot.TurnGyro.resetYaw();

        double currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
        double startTime = getRuntime();
        double runTime;
        targetAngle = targetAngle * -1;

        if (targetAngle < 0) {
            _robot.FrontLeftDrive.setPower(-_turnSpeed);
            _robot.FrontRightDrive.setPower(_turnSpeed);
            _robot.BackLeftDrive.setPower(-_turnSpeed);
            _robot.BackRightDrive.setPower(_turnSpeed);

            while (currAngle > targetAngle) {
                currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
                runTime = getRuntime() - startTime;

                if (runTime > timeout) { break; }

                telemetry.addData("Angle", currAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();
            }
        } else if (targetAngle > 0) {
            _robot.FrontLeftDrive.setPower(_turnSpeed);
            _robot.FrontRightDrive.setPower(-_turnSpeed);
            _robot.BackLeftDrive.setPower(_turnSpeed);
            _robot.BackRightDrive.setPower(-_turnSpeed);

            while (currAngle < targetAngle) {
                currAngle = _robot.TurnGyro.getRobotYawPitchRollAngles().getYaw();
                runTime = getRuntime() - startTime;

                if (runTime > timeout) { break; }

                telemetry.addData("Angle", currAngle);
                telemetry.addData("Target Angle", targetAngle);
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
        _robot.GripperServo.setPosition(0.6);
        sleep(250);
    }

    private void CloseGripper() {
        _robot.GripperServo.setPosition(0.8);
        sleep(250);
    }

    private void LiftHinge() {
        _robot.LeftHingeServo.setPosition(1.0);
        _robot.RightHingeServo.setPosition(0.25);
    }

    private void LowerHinge() {
        _robot.LeftHingeServo.setPosition(0.675);
        _robot.RightHingeServo.setPosition(0.575);
    }

    private int ConvertInchesToTicks(double inches) {
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

    private void InitializeHingeAndClawsPosition(){
        LiftHinge();
        OpenClaw();
        sleep(1000);
        CloseClaw();
        sleep(1000);
        OpenClaw();
        sleep(1000);
    }

    private void InitializeGripper(){
        OpenGripper();
        sleep(1000);
        CloseGripper();
        sleep(1000);
        OpenGripper();
        sleep(1000);
        CloseGripper();
        sleep(1000);
    }
}
