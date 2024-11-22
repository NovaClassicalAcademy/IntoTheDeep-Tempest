
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "NOID", group = "Autonomous")

public class ParkAuto extends LinearOpMode {
  DcMotor FrontLeft;
  DcMotor FrontRight;
  DcMotor BackLeft;
  DcMotor BackRight;
  DcMotor EncoderWheel;
  int TicksPerRotation = 2000;
  double EncoderWheelDiameter = 1.82;
  int StartingPosition;
   /* Code to run ONCE when the driver hits INIT
   */
  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");

    EncoderWheel = hardwareMap.get(DcMotor.class, "EncoderWheel");

    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");

    FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");

    StartingPosition = EncoderWheel.getCurrentPosition();

    waitForStart();
   FrontLeft.setPower(-0.25);
   FrontRight.setPower(0.25);
   BackLeft.setPower(0.25);
   BackRight.setPower(-0.25);
   sleep(500);

   //go forward
    FrontLeft.setPower(0.5);
    FrontRight.setPower(0.5);
    BackLeft.setPower(0.5);
    BackRight.setPower(0.5);

    while (opModeIsActive()){
      double Distance = getDistance();
      if (Distance > 72){
        break;
      }
    }

    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    BackLeft.setPower(0);
    BackRight.setPower(0);

  }

  private double getDistance(){
    double Rotations = (EncoderWheel.getCurrentPosition()- StartingPosition)/TicksPerRotation;
    return Rotations*EncoderWheelDiameter*Math.PI;
  }

}
