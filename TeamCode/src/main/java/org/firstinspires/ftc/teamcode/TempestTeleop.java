
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


@TeleOp(name = "TempestTeleop", group = "TeleOp")

public class TempestTeleop extends OpMode {

  DcMotor FrontLeft;
  DcMotor FrontRight;
  DcMotor BackLeft;
  DcMotor BackRight;

  DcMotor LiftLeft;
  DcMotor LiftRight;
  Double Lift_power = 0.84;

  Servo ServoLeft;
  Servo ServoRight;

  Servo ServoHingeLeft;
  Servo ServoHingeRight;

  Servo ServoGrip;

  Servo ServoDump;

  boolean Hang = false;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    ServoHingeLeft =hardwareMap.get(Servo.class,"HingeLeft");
    ServoHingeRight =hardwareMap.get(Servo.class,"HingeRight");

    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");

    FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
    LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");
    LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);


    ServoLeft = hardwareMap.get(Servo.class, "ServoClawLeft");
    ServoRight = hardwareMap.get(Servo.class, "ServoClawRight");
    ServoGrip = hardwareMap.get(Servo.class, "ServoGrip");
    ServoDump = hardwareMap.get(Servo.class, "ServoDump");

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    telemetry.addData("Status", "Initialized");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
   */
  @Override
  public void init_loop() {
  }

  /*
   * Code to run ONCE when the driver hits START
   */
  @Override
  public void start() {






    ;
  }

  /*
   * Code to run REPEATEDLY after the driver hits START but before they hit STOP
   */
  @Override
  public void loop() {

   //panning motion
    double FL_power = (gamepad1.left_stick_y - gamepad1.left_stick_x)/2;
    double BL_power = (gamepad1.left_stick_y + gamepad1.left_stick_x)/2;
    double FR_power = (gamepad1.left_stick_y + gamepad1.left_stick_x)/2;
    double BR_power = (gamepad1.left_stick_y - gamepad1.left_stick_x)/2;

    //turning motion
    FL_power -= gamepad1.right_stick_x/2;
    BL_power -= gamepad1.right_stick_x/2;
    FR_power += gamepad1.right_stick_x/2;
    BR_power += gamepad1.right_stick_x/2;

    FrontRight.setPower(clamp(FR_power,-1,1));
    FrontLeft.setPower(clamp(FL_power,-1,1));
    BackRight.setPower(clamp(BR_power,-1,1));
    BackLeft.setPower(clamp(BL_power,-1,1));




    if(gamepad2.dpad_left){
      Hang = true;
    }
    else if(gamepad2.dpad_right){
      Hang = false;
    }

    //lift down
    if (gamepad2.dpad_down){
      LiftLeft.setPower(Lift_power);
      LiftRight.setPower(Lift_power);

    }
    //lift up
    else if (gamepad2.dpad_up){
      LiftLeft.setPower(-Lift_power);
      LiftRight.setPower(-Lift_power);
    }
    else if(Hang){
      LiftLeft.setPower(Lift_power);
      LiftRight.setPower(Lift_power);
    }
    else {
      LiftLeft.setPower(0);
      LiftRight.setPower(0);
    }









    telemetry.addData("right trigger", gamepad2.right_trigger);
    telemetry.addData("left trigger", gamepad2.left_trigger);
    telemetry.addData("Lift Left", LiftLeft.getCurrentPosition());
    telemetry.addData("Lift Right", LiftRight.getCurrentPosition());
    telemetry.update();

    //claw full closed
    if (gamepad2.right_bumper) {
      ServoLeft.setPosition(0.44);
      ServoRight.setPosition(0.56);

    }

    // cLaw full open
    else if (gamepad2.left_bumper) {
      ServoLeft.setPosition(0);
      ServoRight.setPosition(1);

    }

    //hinge up
    if (gamepad2.left_trigger > 0.2) {
      ServoHingeRight.setPosition(0.25);
      ServoHingeLeft.setPosition(1);
    }
    //hinge down
    else if (gamepad2.right_trigger > 0.2) {
      ServoHingeRight.setPosition(0.575);
      ServoHingeLeft.setPosition(0.675);
    }
    //hinge middle section
    else if (gamepad2.a) {
      ServoHingeRight.setPosition(0.45);
      ServoHingeLeft.setPosition(0.8);
    }


    //bucket down
    if (gamepad2.y){
      ServoDump.setPosition(1);
    }
    // bucket up
    else if (gamepad2.x){
      ServoDump.setPosition(0.15);
    }
    else if (gamepad1.a){
      ServoDump.setPosition(0.55);
    }

    if(gamepad1.right_bumper){
      ServoGrip.setPosition(0.7);
    }

    else if(gamepad1.left_bumper){
      ServoGrip.setPosition(0.15);
    }







  }


  /*
   * Code to run ONCE after the driver hits STOPp
   */
  @Override
  public void stop() {

    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    BackLeft.setPower(0);
    BackRight.setPower(0);

  }

  private double clamp (double val, double min, double max){
    if (val < min) {
      val = min;
    }
    else if (val > max){
      val = max;
    }
    return val;
  }

}
