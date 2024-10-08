
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TestClawIntake", group = "TeleOp")

public class TempestTeleop extends OpMode {
  /*DcMotor FrontLeft;
  DcMotor FrontRight;
  DcMotor BackLeft;
  DcMotor BackRight;

   */

  Servo ServoLeft;
  Servo ServoRight;

  //double ServoPower = 0.5;








  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
/*
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");

    FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

 */

    ServoLeft = hardwareMap.get(Servo.class, "servoLeft");
    ServoRight = hardwareMap.get(Servo.class, "servoRight");


   // ServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    //ServoRight.setDirection(DcMotorSimple.Direction.FORWARD);




    // Tell the driver that initialization is complete.
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
    /*double FL_power = (gamepad1.left_stick_y - gamepad1.left_stick_x)/2;
    FrontLeft.setPower(FL_power);

    double FR_power = (gamepad1.right_stick_y + gamepad1.right_stick_x)/2;
    FrontRight.setPower(FR_power);

    double BL_power = (gamepad1.left_stick_y + gamepad1.left_stick_x)/2;
    BackLeft.setPower(BL_power);

    double BR_power = (gamepad1.right_stick_y - gamepad1.right_stick_x)/2;
    BackRight.setPower(BR_power);

*/

    telemetry.addData("right trigger", gamepad1.right_trigger);
    telemetry.addData("left trigger", gamepad1.left_trigger);
    telemetry.update();

    if (gamepad1.left_trigger > 0.2) {
      ServoLeft.setPosition(0.25);
      ServoRight.setPosition(0.25);



      
    }
    /*

    if (gamepad1.right_trigger > 0.2) {
      ServoLeft.setPosition(0);
      ServoRight.setPosition(0);
    }


     */


  }

  /*
   * Code to run ONCE after the driver hits STOPp
   */
  @Override
  public void stop() {
    /*
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    BackLeft.setPower(0);
    BackRight.setPower(0);
*/
  }

}
