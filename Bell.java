package org.firstinspires.ftc.teamcode;

import java.util.Calendar;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Bell", group = "Autonomous")
public class Bell extends OpMode {

    private final double BELL_SPEED = 1;
    private final double TIME_PER_RUN = 30;
    

    private DcMotor l;
    private DcMotor r;
    private DcMotor b;
    
    private DistanceSensor E1;
    private DistanceSensor DF;
    private DistanceSensor DB;
    
    private LED led;
    
    private boolean[] ran = new boolean[4];
    private int numberRan = 0;
    
    private ElapsedTime moveTimer = new ElapsedTime();
    private boolean isMoving = false;
    private int direction = 1;
    
    enum State { WAITING, RUNNING_ACTION, DONE }

    State state = State.WAITING;

    final double BASE_POWER = 0.15;
    
    @Override
    public void init() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        b = hardwareMap.dcMotor.get("b");
        
        led = hardwareMap.get(LED.class, "L1");
        

        E1 = hardwareMap.get(DistanceSensor.class, "E1");
        DF = hardwareMap.get(DistanceSensor.class, "DF");
        DB = hardwareMap.get(DistanceSensor.class, "DB");

        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        b.setDirection(DcMotorSimple.Direction.FORWARD);
        led.off();
    }

    @Override
    public void loop() {
        Calendar now = Calendar.getInstance();
        int hour = now.get(Calendar.HOUR_OF_DAY);
        int minute = now.get(Calendar.MINUTE);
        int second = now.get(Calendar.SECOND);
        
        if (E1.getDistance(DistanceUnit.CM) > 10 || E1.getDistance(DistanceUnit.CM) < 3) {
            led.on();
            requestOpModeStop();
        }
        switch (state) {
            case WAITING:
                if (hasReachedTime(hour, minute, second, 14, 0, 0) && (!ran[0])
                || hasReachedTime(hour, minute, second, 15, 55, 0) && (!ran[1])
                || hasReachedTime(hour, minute, second, 23, 0, 0) && (!ran[2])
                || hasReachedTime(hour, minute, second, 23, 0, 30) && (!ran[3])) {
                    state = State.RUNNING_ACTION;
                }
                telemetry.addData("State", state);
                telemetry.addData("Time", String.format("%02d:%02d:%02d", hour, minute, second));
                telemetry.addData("numberRan", numberRan);
                telemetry.addData("front sensor", DF.getDistance(DistanceUnit.CM));
                telemetry.addData("back sensor", DB.getDistance(DistanceUnit.CM));
                telemetry.addData("emergency", E1.getDistance(DistanceUnit.CM));
                telemetry.update();
                break;

            case RUNNING_ACTION:
                if (!isMoving) {
                    moveTimer.reset();
                    isMoving = true;
                }
                
            

                if (moveTimer.seconds() < TIME_PER_RUN) {
                    double leftPower = BASE_POWER * direction, rightPower = BASE_POWER * direction;
                    if (direction == -1) {
                        leftPower -= 0.04;
                    } else if (direction == 1) {
                        leftPower += 0.04;
                    }
                    double front_dist = DF.getDistance(DistanceUnit.CM);
                    double back_dist = DB.getDistance(DistanceUnit.CM);
                    double diff = Math.abs(Math.abs(front_dist - back_dist) - 0.2);
                    
                    if (front_dist < back_dist) {
                        if (direction == 1) {
                            rightPower += diff / 5;
                        } else {
                            leftPower -= diff / 5;
                        }
                    } else {
                        if (direction == 1) {
                            leftPower += diff / 5;
                        } else {
                            rightPower -= diff / 5;
                        }
                        
                    }
                    
                    leftPower = Math.min(Math.max(leftPower, -1), 1);
                    rightPower = Math.min(Math.max(rightPower, -1), 1);
                    l.setPower(leftPower);
                    r.setPower(rightPower);
                    telemetry.addData("left", leftPower);
                    telemetry.addData("right", rightPower);
                    if (direction == 1) {
                        if (leftPower > rightPower) {
                            telemetry.addData("dir", "moving left");
                        } else {
                            telemetry.addData("dir", "moving right");
                        }
                    } else {
                        if (leftPower < rightPower) {
                            telemetry.addData("dir", "moving left");
                        } else {
                            telemetry.addData("dir", "moving right");
                        }
                    }
                    telemetry.addData("front sensor", DF.getDistance(DistanceUnit.CM));
                    telemetry.addData("back sensor", DB.getDistance(DistanceUnit.CM));
                    telemetry.addData("emergency", E1.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    b.setPower(-1 * BELL_SPEED);
                } else {
                    l.setPower(0);
                    r.setPower(0);
                    b.setPower(0);
                    ran[numberRan] = true;
                    numberRan++;
                    direction *= -1;
                    if (numberRan == 4) {
                        state = State.DONE;
                    } else {
                        state = State.WAITING;
                    }
                    isMoving = false;
                }
                break;
            case DONE:
                break;
        } 
    }
    private boolean hasReachedTime(int hNow, int mNow, int sNow, int hTarget, int mTarget, int sTarget) {
        return (hNow > hTarget ||
            (hNow == hTarget && (mNow > mTarget || (mNow == mTarget && sNow >= sTarget)))) &&
            (hNow < hTarget ||
            (hNow == hTarget && (mNow < mTarget + 1 || (mNow == mTarget + 1 && sNow < sTarget))));
        }
    }
