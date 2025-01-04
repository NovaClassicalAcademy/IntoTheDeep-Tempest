package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Rei Telemetry Read Out", group = "Rei")
public class TelemetryReadOut extends LinearOpMode {
    HardwareRobot _robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        _robot.Init(hardwareMap);

        ResetGripper();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Initialization", "Complete");
            telemetry.addData("Front Left Drive", _robot.FrontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Drive", _robot.FrontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Drive", _robot.BackLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Drive", _robot.BackRightDrive.getCurrentPosition());
            telemetry.addData("Encoder Wheel Drive", _robot.YEncoderWheel.getCurrentPosition());
            telemetry.addData("L Lift Position", _robot.LeftLiftMotor.getCurrentPosition());
            telemetry.addData("R Lift Position", _robot.RightLiftMotor.getCurrentPosition());
            telemetry.addData("Gripper Position", _robot.GripperServo.getPosition());
            telemetry.addData("R Claw Position", _robot.RightClawServo.getPosition());
            telemetry.addData("L Claw Position", _robot.LeftClawServo.getPosition());
            telemetry.addData("R Hinge Position", _robot.RightHingeServo.getPosition());
            telemetry.addData("L Hinge Position", _robot.LeftHingeServo.getPosition());
            telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());
            telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());
            telemetry.addData("Lift Position", _robot.LeftLiftMotor.getCurrentPosition());
            telemetry.addData("Dump Position", _robot.DumpBucketServo.getPosition());

            telemetry.update();
        }

    }

    private void ResetGripper(){
        _robot.GripperServo.setPosition(0.8);
    }
}
