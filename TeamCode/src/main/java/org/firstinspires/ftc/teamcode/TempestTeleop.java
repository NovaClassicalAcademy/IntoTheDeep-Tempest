

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

//OK LEO TES


@TeleOp(name = "TempestTeleop", group = "TeleOp")


public class TempestTeleop extends OpMode {


  DcMotorEx FrontLeft;
  DcMotorEx FrontRight;
  DcMotorEx BackLeft;
  DcMotorEx BackRight;


  DcMotorEx LiftLeft;
  DcMotorEx LiftRight;


  Servo ServoClaw;
  Servo ServoPitch;
  Servo ServoRoll;


  Servo ServoHingeLeft;
  Servo ServoHingeRight;


  Servo ServoGrip;


  Servo ServoDump;

int LiftPower = 8;
  boolean Hang = false;



  ElapsedTime Timer;
/*

  double IntegralSum = 0;
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double lastError = 0;



  double LiftPower = PIDControl(1000, LiftRight.getVelocity());


  public double PIDControl(double reference, double state){
    double error = reference - state;
    IntegralSum += error * Timer.seconds();
    double derivative = (error - lastError) / Timer.seconds();
    lastError = error;




    Timer.reset();
    double output = (error * kp) + (derivative * ki) + (IntegralSum * kd);
    return output;
  }
*/



  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    Timer = new ElapsedTime();



    ServoHingeLeft =hardwareMap.get(Servo.class,"HingeLeft");
    ServoHingeRight =hardwareMap.get(Servo.class,"HingeRight");


    FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");


    FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);


    BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);


    LiftLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
    LiftRight = hardwareMap.get(DcMotorEx.class, "LiftRight");
    LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);


    //LiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //LiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    ServoClaw = hardwareMap.get(Servo.class, "ServoClaw");
    ServoRoll = hardwareMap.get(Servo.class, "WristRoll");
    ServoPitch = hardwareMap.get(Servo.class, "WristPitch");


    ServoClaw.setDirection(Servo.Direction.REVERSE);


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








    if(gamepad2.left_stick_button){
      Hang = true;
    }
    else if(gamepad2.right_stick_button){
      Hang = false;
    }


    //lift down
    if (gamepad2.dpad_down){
      LiftLeft.setPower(LiftPower);
      LiftRight.setPower(LiftPower);


    }
    //lift up
    else if (gamepad2.dpad_up){
      LiftLeft.setPower(-LiftPower);
      LiftRight.setPower(-LiftPower);
      ServoPitch.setPosition(0.9);
    }
    else if(Hang){
      LiftLeft.setPower(LiftPower);
      LiftRight.setPower(LiftPower);
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

    /*
    CLaw stuff
    double servoRollCurrentPosition = ServoRoll.getPosition();
    double servoPitchCurrentPosition = ServoPitch.getPosition();
    double servoSensitivity = 0.5;


    //claw full closed
    if (gamepad2.right_bumper) {
      ServoClaw.setPosition(1);
    }
    // cLaw full open
    else if (gamepad2.left_bumper) {
      ServoClaw.setPosition(0.5);
    }
    //Claw Rotation
    ServoRoll.setPosition(servoRollCurrentPosition + (gamepad2.left_stick_x * servoSensitivity));
    ServoPitch.setPosition(servoPitchCurrentPosition + (gamepad2.left_stick_y * servoSensitivity));


    //Claw Deposit Position
    if(gamepad2.left_trigger > 0.2){
      ServoHingeRight.setPosition(0.25);
      ServoHingeLeft.setPosition(1);


      ServoRoll.setPosition(0.5);//Middle
      ServoPitch.setPosition(1);//All the way back. Find them out
    }



*/
    //claw open
    if(gamepad2.left_bumper){
      ServoClaw.setPosition(0.4);
    }
    //claw closed
    if(gamepad2.right_bumper){
      ServoClaw.setPosition(0.85);
    }


    //Roll movements
    //(axon faces)1:30
    if(gamepad2.left_stick_x > 0.3){//right
      ServoRoll.setPosition(0.65);
    }
    //3:00
    else if(gamepad2.left_stick_y > 0.3){//down
      ServoRoll.setPosition(0.25);
    }
    //11:30
    else if(gamepad2.left_stick_x < -0.3){//left
      ServoRoll.setPosition(0.35);
    }
    //12:00
    else if(gamepad2.left_stick_y < -0.3){//up
      ServoRoll.setPosition(0.51);
    }

    //Pitch movements
    if(gamepad2.right_stick_y > 0.1){//down
      ServoPitch.setPosition(0.2);
    }
    else if(gamepad2.right_stick_y < -0.1){//up
      ServoPitch.setPosition(0.9);
    }


    //hinge up
    if (gamepad2.left_trigger > 0.2) {
      ServoHingeRight.setPosition(0.350);
      ServoHingeLeft.setPosition(0.9);//CHECK THIS OUT GOTTA UPLOAD NEW POS
      ServoPitch.setPosition(0.2);
      ServoRoll.setPosition(0.5);

    }
    //hinge down
    else if (gamepad2.right_trigger > 0.2) {
      ServoHingeRight.setPosition(0.585);
      ServoHingeLeft.setPosition(0.665);
      ServoPitch.setPosition(0.9);
    }
    //hinge middle section
    else if (gamepad2.a) {
      ServoHingeRight.setPosition(0.535);
      ServoHingeLeft.setPosition(0.705);
    }
    /*
    When the left value goes up the right value goes down
     */




    //bucket down
    if (gamepad2.y){
      ServoDump.setPosition(1);
    }
    // bucket up
    else if (gamepad2.x){
      ServoDump.setPosition(0.15);
    }
    //bucket hang
    else if (gamepad1.a){
      ServoDump.setPosition(0.55);
    }


    //grip close
    if(gamepad1.right_bumper){
      ServoGrip.setPosition(0.85);
    }
    //grip open
    else if(gamepad1.left_bumper){
      ServoGrip.setPosition(0.15);
    }


    //Limited Extension
    int LiftRightPosition = LiftRight.getCurrentPosition();

    if(LiftRightPosition < -1500) {
      ServoHingeRight.setPosition(0.35);
      ServoHingeLeft.setPosition(0.9);
      ServoPitch.setPosition(1);
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
