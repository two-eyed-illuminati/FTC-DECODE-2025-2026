import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTest extends OpMode{
    DcMotorEx Motor1;
    DcMotorEx Motor2;

    @Override
    public void init(){
        Motor1 = hardwareMap.get(DcMotorEx.class,"motor1");
        Motor2 = hardwareMap.get(DcMotorEx.class,"motor2");
        Motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        double power = gamepad1.left_stick_y;
        Motor1.setPower(power);
        Motor2.setPower(power);
        telemetry.addData("power:", power);
    }
}