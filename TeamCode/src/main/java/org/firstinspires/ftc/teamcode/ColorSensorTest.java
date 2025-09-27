import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp
public class ColorSensorTest extends OpMode {
    NormalizedColorSensor Color;

    @Override
    public void init(){
        Color = hardwareMap.get(NormalizedColorSensor.class,"color1");

    }
    @Override
    public void loop(){
        telemetry.addData("colorr:", Color.getNormalizedColors().red);
        telemetry.addData("colorg:", Color.getNormalizedColors().green);
        telemetry.addData("colorb:", Color.getNormalizedColors().blue);
    }
}
