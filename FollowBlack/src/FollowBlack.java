import lejos.nxt.LightSensor;

public class FollowBlack extends Robotabor {

	private static int hranice;

	public static void main(String[] args) {
		
		inicializuj();
		kalibrace();
		jedNaCernou();
		following();
		search();

	}

	private static void kalibrace() {
		print("Bila");
		int bila = light1.getNormalizedLightValue();
		getButton();
		print("Cerna");
		int cerna = light1.getNormalizedLightValue();
		getButton();
		hranice = (bila + cerna)/2;

	}

	private static void following() {
		buggy.forward();
		while(!jsemNaBile());
		search();

	}

	private static void search() {
		buggy.speed(50);
		buggy.turn(2);
		int i = light1.getLightValue();
		buggy.turn(-2);
		int y = light1.getLightValue();

		if(!jsemNaBile()){
			following();
		}
		else{
			while(i > y){
				buggy.turn(2);
				buggy.forward();

			}
			while(i < y){
				buggy.turn(-2);
				buggy.forward();
			}
		}
	}

	private static void inicializuj() {
		init(SENSOR.NONE, SENSOR.NONE, SENSOR.NONE, SENSOR.LIGHT);
		initBuggy(56, 110);
		light1.setFloodlight(true);
		getButton();

	}
	private static void jedNaCernou() {
		buggy.speed(100);
		buggy.forward();
		while (jsemNaBile());
		buggy.stop();
	}

	private static boolean jsemNaBile() {
		return light1.getNormalizedLightValue() > 700;
	}
}
