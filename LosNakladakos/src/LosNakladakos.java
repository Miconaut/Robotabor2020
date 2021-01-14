
public class LosNakladakos extends Robotabor {
	private static int bila;
	private static int cerna;
	private static int hranice;
	private static float speed = 100;
	private static float speed2 = 50;

	public static void main(String[] args) {

		inicializuj();
		kalibrace();
		calibrateWhite();
		getButton();
		beep();

		prejezd();
		jedNaCernou();
		mezijizda();
		beep();
		jedNaCernou();
		beep();
		prejezd();
		search();
		toceni();


		while (true){
			following();
			nakladani();
			couvani();
			otocka();
			following();
			vykladani();
			couvani();
			otocka();
		}

	}

	private static void couvani() {
		buggy.speed(speed);
		buggy.go(-120);
		buggy.stop();
	}

	private static void toceni() {
		buggy.turn(-30);
	}

	private static void otocka() {
		buggy.turn(-180);
	}

	private static void vykladani() {
		motA.setSpeed(50);
		motA.rotate(60);
		sleepMilliseconds(20);
		motA.rotate(-60);
		sleepMilliseconds(500);
		beep();
	}

	private static void nakladani() {
		sleepMilliseconds(15000);
		beep();
	}

	private static void prejezd() {
		buggy.speed(speed);
		buggy.go(60);
		buggy.stop();
	}

	private static void calibrateWhite() {
		getButton();
		//buggy.acceleration(10);
		buggy.speed(20);
		follower.calibrateBuggy(60);
		beep();
	}

	private static void mezijizda() {
		buggy.speed(speed);
		buggy.go(120);
		//buggy.stop();
	}

	private static void kalibrace() {
		print("Bila \n");
		getButton();
		bila = light1.getNormalizedLightValue();
		beep();
		print("Cerna \n");
		getButton();
		cerna = light1.getNormalizedLightValue();
		hranice = (bila + cerna) / 2;
		print("BLM=");
		print(cerna);
		print(" WLM=");
		print(bila);
		print("\n");
		getButton();
		beep();

	}

	private static void following() {
		buggy.speed(speed2);
		buggy.forward();
		//follower.findTrack(20);
		follower.startFollowing(1);
		while (!touch1.isPressed()){
			follower.follow();
		}
		beep();
		beep();
		follower.stopFollowing();
		buggy.stop();
		beep();
	}

	private static void search() {
		buggy.speed(speed2);

		while (jsemNaBile()) {
			buggy.turn(-4);
		}

		/*
		 * int i = otockaVlevoAMereni(); int y = otockaVpravoAMereni();
		 *
		 * if(jsemNaBile()){ while(i > y){ otockaVlevoAMereni();
		 *
		 * } while(i < y){ buggy.turn(-2); buggy.forward(); } }
		 */

	}


	private static void inicializuj() {
		init(SENSOR.TOUCH, SENSOR.NONE, SENSOR.NONE, SENSOR.LIGHT);
		initBuggy(56, 110);
		initFollower(light1);
		light1.setFloodlight(true);

	}

	private static void jedNaCernou() {
		// buggy.acceleration(30); // testik
		buggy.speed(speed);
		buggy.forward();
		while (jsemNaBile());
		buggy.stop();

	}

	private static boolean jsemNaBile() {
		return light1.getNormalizedLightValue() > hranice;
	}
}