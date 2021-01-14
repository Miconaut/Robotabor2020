import lejos.nxt.LightSensor;

public class LightInfo extends Robotabor {

	public static void main(String[] args) {
		init(SENSOR.NONE, SENSOR.NONE, SENSOR.NONE, SENSOR.LIGHT);
		light1.setFloodlight(true);
		getButton();
		svetylko();
	}

	private static void svetylko() {
		print(light1.getNormalizedLightValue() + ", ");
		getButton();
		svetylko();
	}

}
