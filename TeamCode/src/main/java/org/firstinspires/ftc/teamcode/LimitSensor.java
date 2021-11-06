package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LimitSensor extends Component {

    public TouchSensor digitalTouch;

    public LimitSensor(int port, String name, HardwareMap map) {
        super(port, name);
        this.digitalTouch = map.touchSensor.get(name);
    }

    public boolean isPressed(){
        return digitalTouch.isPressed();
    }

}
