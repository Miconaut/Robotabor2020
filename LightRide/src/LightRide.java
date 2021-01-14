import lejos.nxt.LightSensor;

public class LightRide extends Robotabor {

	public static void main(String[] args) {
		init(SENSOR.NONE, SENSOR.NONE, SENSOR.NONE, SENSOR.LIGHT);
		initBuggy(56, 110);
		buggy.speed(100);
		naJakeBarveJsem();
		jedNaCernou();
		getButton();
	}

	private static void jedNaCernou() {
		buggy.forward();
		while (light1.getNormalizedLightValue() > 350);
		buggy.stop();
	}
	/*
	 * float i = light1.getLightValue(); int barva = 350;
	 *
	 * if (i > barva){ print("White"); } else{ print("Black"); }
	 */

	private static void naJakeBarveJsem() {
		light1.setFloodlight(true);
		if (jsemNaBile()) {
			print("White");
		}
		else {
			print("Black");
		}

	}

	private static boolean jsemNaBile() {
		// TODO Auto-generated method stub
		return light1.getNormalizedLightValue() > 350;
	}

}
