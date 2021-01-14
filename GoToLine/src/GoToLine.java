import lejos.nxt.LightSensor;

public class GoToLine extends Robotabor {

	public static void main(String[] args) {
		inicializuj();
		jedNaCernou();
		beep();
		getButton();
		
	}

	private static void inicializuj() {
		init(SENSOR.NONE, SENSOR.NONE, SENSOR.NONE, SENSOR.LIGHT);
		initBuggy(56, 110);
		buggy.speed(100);
		light1.setFloodlight(true);
		getButton();

	}
	private static void jedNaCernou() {
		buggy.forward();
		while (jsemNaBile());
		buggy.stop();
	}

	private static boolean jsemNaBile() {
		return light1.getNormalizedLightValue() > 400;
	}

}
