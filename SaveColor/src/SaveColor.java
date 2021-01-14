
public class SaveColor extends Robotabor {
	private static int bila;
	private static int cerna;
	private static int hranice;

	public static void main(String[] args) {

		inicializuj();
		kalibrace();
		vypis();
		/*jedNaCernou();
		following();
		search();*/

	}

	private static void vypis() {
		while(true){
			if(jsemNaBile()){
				print("Bila \n");
			}
			else{
				print("Cerna \n");
			}
		}
	}

	private static void kalibrace() {
		print("Bila \n");
		getButton();
		bila = light1.getNormalizedLightValue();
		getButton();
		print("Cerna \n");
		getButton();
		cerna = light1.getNormalizedLightValue();
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
		//getButton();

	}
	private static void jedNaCernou() {
		buggy.speed(100);
		buggy.forward();
		while (jsemNaBile());
		buggy.stop();
	}

	private static boolean jsemNaBile() {
		return light1.getNormalizedLightValue() > hranice;
	}
}
