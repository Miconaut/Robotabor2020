import lejos.nxt.*;
import lejos.util.Stopwatch;

public abstract class Robotabor {
	public static class NXTRegMotor extends NXTRegulatedMotor {
		private int last_state = 0; // -1=backward 0=stop 1=forward

		/**
		 * Vytvori instanci motoru
		 *
		 * @param b
		 *            port ke kteremu je pripojeny
		 */
		public NXTRegMotor(MotorPort b) {
			super(b);
			last_state = 0;
		}

		/**
		 * Nastav pozadovanou rychlost toceni motoru
		 *
		 * @param degPerSecond
		 *            rychlost v stupnich za sekundu
		 */
		public void setSpeed(int degPerSecond) {
			if (degPerSecond == 0) {
				if (last_state != 0) {
					last_state = 0;
					super.stop(true);
				}
			} else if (degPerSecond > 0) {
				if (last_state <= 0)
					forward();
				super.setSpeed(degPerSecond);
				last_state = 1;
			} else {
				if (last_state >= 0)
					backward();
				super.setSpeed(-degPerSecond);
				last_state = -1;
			}
		}

		/**
		 * Nastav pozadovanou rychlost toceni motoru -- nepouzivat, neupravena
		 * puvodni funkce
		 *
		 * @param degPerSecond
		 *            pozadovana rychlost ve stupnich za sekundu
		 */
		public void origsetSpeed(int degPerSecond) {
			super.setSpeed(degPerSecond);
		}

		/**
		 * Zastav motor brzdenim
		 *
		 * @param immediate
		 *            true: hned vyskocit z funkce, false: blokovat dokud
		 *            nedojde k zastaveni
		 */
		public void stop(boolean immediate) {
			last_state = 0;
			super.stop(immediate);
		}

		/**
		 * Zastav motor, nech prirozene dojet
		 */
		public void flt() {
			last_state = 0;
			super.flt();
		}
	}

	private static Stopwatch _TT;

	/**
	 * Vytiskni hodnotu
	 *
	 * @param b
	 *            co chci vytisknout
	 */
	public static void print(boolean b) {
		System.out.print(b);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param c
	 *            co chci vytisknout
	 */
	public static void print(char c) {
		System.out.print(c);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param i
	 *            co chci vytisknout
	 */
	public static void print(int i) {
		System.out.print(i);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param l
	 *            co chci vytisknout
	 */
	public static void print(long l) {
		System.out.print(l);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param f
	 *            co chci vytisknout
	 */
	public static void print(float f) {
		System.out.print(f);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param d
	 *            co chci vytisnkout
	 */
	public static void print(double d) {
		System.out.print(d);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param a
	 *            co chci vytisknout
	 */
	public static void print(char[] a) {
		System.out.print(a);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param s
	 *            co chci vytisknout
	 */
	public static void print(String s) {
		System.out.print(s);
	}

	/**
	 * Vytiskni hodnotu
	 *
	 * @param o
	 *            co chci vytisknout
	 */
	public static void print(Object o) {
		System.out.print(o);
	}

	/**
	 * Cekej nejakou dobu
	 *
	 * @param miliseconds
	 *            pocet milisekund, kolik cekat
	 * @return 0, kdyz to dopadlo dobre, -1 pri chybe
	 */
	public static int sleepMilliseconds(int miliseconds) {
		try {
			Thread.sleep(miliseconds);
		} catch (InterruptedException e) {
			return -1;
		}
		return 0;
	}

	/**
	 * dej procesoru sanci, aby se venoval taky necemu dalsimu
	 */
	public static void yield() {
		Thread.yield();
	}

	/**
	 * Zjisti, kolik ubehlo casu od zacatku behu
	 *
	 * @return pocet milisekund od zacatku
	 */
	public static int elapsedMilliseconds() {
		return _TT.elapsed();
	} /* returns current time in milliseconds */

	/**
	 * Pockej na zmacknuti tlacitka a vrat, co bylo zmacknuto
	 *
	 * @return soucet toho, co bylo zmacknuto... 0=nic, 1=enter, 2=left,
	 *         4=right, 8=escape
	 */
	public static int getButton() {
		Button.waitForAnyPress();
		return readButton();
	}

	/**
	 * Vrat, co se aktualne macka
	 *
	 * @return soucet toho, co bylo zmacknuto... 0=nic, 1=enter, 2=left,
	 *         4=right, 8=escape
	 */
	public static int readButton() {
		return Button.readButtons();
	} /* 1=ENTER | 2=LEFT | 4=RIGHT | 8=ESCAPE */

	/**
	 * pipni
	 */
	public static void beep() {
		Sound.beep();
	}

	public static void playTone(int freq, int durationMilliseconds, int volume){
		Sound.playTone(freq, durationMilliseconds, volume);
	}

	/**
	 * motory pripojene k jednotlivym portum
	 */
	public static NXTRegMotor motA, motB, motC;

	/*
	 * * Priklad pouziti sensoru: TOUCH: touch1.isPressed() vrati true kdyz je
	 * tlacitko stlaceno, jinak false SONAR: sonar.getDistance() vrati 0 az 255
	 * (255 znamena prilis velka vzdalenost nebo chyba)
	 *
	 * Je mozne mit az 4 tlacika touch1 az touch4, az 2 svetelne sensory light1
	 * a light2 a jeden sonar. Jejich prirazeni portum se urcuje ve funkci init,
	 * napr. init(SNS.LIGHT,SNS.TOUCH,SNS.LIGHT,SNS.SONAR); priradi svetlo na
	 * port 1 (light1) a 3 (light2), hmat na port 2 (touch1) a sonar na port 4
	 * (sonar). V zavorce je vzdy napsana cast pred teckou pri pouzivani.
	 *
	 * Priklad pouziti motoru: Bud se da pouzit init_buggy(polomer_kola,
	 * rozchod_kol) pro motory B a C. Pak lze pouzivat go(vzdalenost_v_mm),
	 * turn(otoveni_vlevo_ve_stupnich), speed(rychlost_v_mm_za_sekundu) a
	 * reduce_acceleration() pro hladky rozjezd a brzdeni, nebo je mozne ridit
	 * motory primo. Jsou pristupne pres motA, motB a motC. Rizeni ukazu na
	 * motA: motA.rotate(x,false) otoci motor o x stupnu (muze byt zaporne i
	 * vetsi nez 360) motA.rotate(x,true) totez ale neceka na dokonceni -
	 * program bezi dal motA.rotateTo(x,false) natoci motor na polohu x stupnu
	 * od zacatku (muze byt zaporne) motA.rotateTo(x,true) totez, ale neceka na
	 * dokonceni motA.isMoving() vrati true pokud se motor pusteny pomoci
	 * rotate(x,true) porad jeste snazi hybat motA.setSpeed(v) nastavi rychlost
	 * otaceni na v stupnu za sekundu motA.stop() okamzite zastavi motor a drzi
	 * ho silou motA.flt() zastavi motor a necha ho dotocit - motor zustane
	 * volne motA.getTachoCount() vrati aktualni polohu motoru ve stupnich. Ve
	 * spojeni s rotateTo(x,true) jde pouzit ke zmereni zateze motoru podle
	 * rychlosti zaberu
	 */
	public enum SENSOR {
		/**
		 * zadny sensor
		 */
		NONE, TOUCH,
		/**
		 * <h1>Priklad pouziti</h1>
		 *
		 * Pristup k prvnimu sensoru pres light1, k dalsim s vyssim cislem (jde
		 * o pocet pouzitych stejnych sensoru, ne o zapojeni do portu). </br>
		 * </br>
		 *
		 * <b>light1.readNormalizedValue()</b> vrati 0 (tma) az 1023 (nejvetsi
		 * svetlo) <br/>
		 * <b>light1.setFloodlight(true)</b> rozsviti cervene svetlo<br/>
		 * <b>light1.setFloodlight(false)</b> zhasne cervene svetlo<br/>
		 */
		LIGHT, SONAR
	};

	private static int _light_ct, _touch_ct, _sonar_ct;
	public static LightSensor light1, light2;
	public static UltrasonicSensor sonar, sonar1, sonar2;
	public static TouchSensor touch1, touch2, touch3, touch4;

	private static void attachSensor(SENSOR in, SensorPort sp) {
		if (in != SENSOR.NONE) {
			if (SENSOR.TOUCH == in) {
				TouchSensor t = new TouchSensor(sp);
				switch (_touch_ct) {
				case 1:
					touch1 = t;
					break;
				case 2:
					touch2 = t;
					break;
				case 3:
					touch3 = t;
					break;
				default:
					touch4 = t;
				}
				_touch_ct++;
			} else if (SENSOR.LIGHT == in) {
				LightSensor l = new LightSensor(sp);
				switch (_light_ct) {
				case 1:
					light1 = l;
					break;
				default:
					light2 = l;
				}
				_light_ct++;
			} else {
				UltrasonicSensor s = new UltrasonicSensor(sp);
				switch (_sonar_ct) {
				case 1:
					sonar = s;
					sonar1 = s;
					break;
				default:
					sonar2 = s;
				}
				_sonar_ct++;
			}
		}
	}

	/**
	 * inicializuj knihovnu bez pripojenych sensoru
	 */
	public static void init() {
		init(SENSOR.NONE, SENSOR.NONE, SENSOR.NONE, SENSOR.NONE);
	}

	/**
	 * Inicializace knihovny s pripojenim sensoru.
	 *
	 *
	 * @param p1
	 *            sensor pripojeny k portu 1
	 * @param p2
	 *            sensor pripojeny k portu 2
	 * @param p3
	 *            sensor pripojeny k portu 3
	 * @param p4
	 *            sensor pripojeny k portu 4
	 */
	public static void init(SENSOR p1, SENSOR p2, SENSOR p3, SENSOR p4) {
		print("EasyRobotLibrary v 2020.10\n");
		_TT = new Stopwatch();
		_TT.reset();
		motA = new NXTRegMotor(MotorPort.A);
		motA.flt();
		motB = new NXTRegMotor(MotorPort.B);
		motB.flt();
		motC = new NXTRegMotor(MotorPort.C);
		motC.flt();
		_light_ct = 1;
		_touch_ct = 1;
		_sonar_ct = 1;
		attachSensor(p1, SensorPort.S1);
		attachSensor(p2, SensorPort.S2);
		attachSensor(p3, SensorPort.S3);
		attachSensor(p4, SensorPort.S4);
	}

	/**
	 * Inicializuj rizeni motoru pro robota, predpoklada se ze C je levy motor,
	 * B je pravy motor
	 *
	 * @param wheelDiameter
	 *            prumer kola
	 * @param wheelDistance
	 *            vzdalenost mezi koly
	 */
	public static void initBuggy(float wheelDiameter, float wheelDistance) {
		initBuggy(wheelDiameter, wheelDistance, motC, motB);
	}

	/**
	 * pristup k rizeni celeho robota
	 */
	public static Buggy buggy;

	/**
	 * Inicializuj rizeni motoru pro robota
	 *
	 * @param wheelDiameter
	 *            prumer kola
	 * @param wheelDistance
	 *            vzdalenost mezi koly
	 * @param leftMotor
	 *            ke kteremu portu je pripojen levy motor (napr. motC)
	 * @param rightMotor
	 *            ke kteremu portu je pripojen pravy motor (napr. motB)
	 */
	public static void initBuggy(float wheelDiameter, float wheelDistance, NXTRegMotor leftMotor,
			NXTRegMotor rightMotor) {
		initBuggy(wheelDiameter, wheelDistance, leftMotor, rightMotor, false, false);
	}

	public static void initBuggy(float wheelDiameter, float wheelDistance, NXTRegMotor leftMotor,
			NXTRegMotor rightMotor, boolean reverseLeft, boolean reverseRight) {
		buggy = new Buggy(wheelDiameter, wheelDistance, leftMotor, rightMotor, reverseLeft, reverseRight);
	}

	private static int last_time;
	private static LightSensor follow_ls;
	private static float last_e, acc_e;
	public static Follower follower;


	/**
	 * Inicializuje modul sledovani cary (musi byt nastaven pohyb pomoci initBuggy).
	 * K funkcim na sledovani cary se pak pristupuje pomoci follower.\*
	 * @param ls svetelny sensor, ktery se bude pouzivat ke sledovani cary, napr. light1
	 */
	public static void initFollower(LightSensor ls){
		follower = new Follower(ls);
	}

	private static float forward_speed;


	public static class Follower {

		private static int black_level, white_level;
		private static float follow_p, follow_i, follow_d;
		private LightSensor ls;

		public Follower(LightSensor ls){
			this.ls = ls;
		}

		/**
		 * Rekni jak moc jsi na cerne nebo na bile
		 *
		 * @param ls
		 *            ktery light sensor pouzit (napr. light1)
		 * @return vrati -1 (jsme kompletne na cerne) az 1 (jsme kompletne na bile)
		 */
		public static float getOfftrackValue(LightSensor ls) {
			int c = ls.readNormalizedValue();
			return (2 * (float) (c - black_level)) / (white_level - black_level) - 1;
		}

		/**
		 * Kalibruj svetelny sensor robota
		 *
		 * Polozit robota cidlem na bilou, vpravo od cerne (cidlo je na predku
		 * robota, jinak obracne) tak aby cervene svetlo promitane na zem bylo zcela
		 * na bile a svym okrajem bylo blizko cerne. Kalibrace zacne otacet robotem
		 * vlevo o zadany uhel, cimz prejede od bile do cerne a zmeri si minimum a
		 * maximum, ktere pak bude pouzivat. Je vhodne pro tuto operaci nastavit
		 * mensi rychlost a zrychleni pomoci speed() a reduce_acceleration() a pak
		 * je zase treba zvysi pro jizdu.
		 *
		 * @param ls
		 *            ktery light sensor pouzit (napr. light1)
		 * @param angleStep
		 *            uhel o ktery se v jednom kroku otoci robot, kdyz hleda cernou
		 *            a bilou
		 */
		public void calibrateBuggy(int angleStep) {
			buggy.turn(angleStep, true);
			int pokracovat = 2;
			follow_ls = ls;
			black_level = 65536;
			white_level = 0;
			while (pokracovat > 0) {
				int jas = ls.readNormalizedValue();
				if (jas < black_level)
					black_level = jas;
				if (jas > white_level)
					white_level = jas;
				if (!buggy.isGoing()) {
					if (pokracovat > 1) {
						pokracovat = 1;
						buggy.turn(-angleStep, true);
					} else {
						pokracovat = 0;
					}
				}
			}
			if (white_level <= black_level) {
				beep();
				sleepMilliseconds(120);
				print("selhani kalibrace\n");
				beep();
				sleepMilliseconds(20);
				beep();
				getButton();
				getButton();
			}
			print("B=");
			print(black_level);
			print(" W=");
			print(white_level);
			print("\n");
		}

		public void recalibrateOnWhite() {
			int posunuti = follow_ls.readNormalizedValue() - white_level;
			white_level = white_level + posunuti;
			black_level = black_level + posunuti;
		}

		/**
		 * Zacni sledovat caru. Sledovani cary neovlivnuje rychlost a zrychleni robota,
		 * je vhodne ji predtim nastavit pomoci buggy.speed, prip. buggy.acceleration.
		 * Po zavolani teto funkce je treba pak stale dokola volat follower.follow(),
		 * kdyz chceme caru prestat sledovat, zvolime follower.stopFollowing() a nasledne
		 * buggy.stop() (nebo jine zastaveni robota).
		 * @param p parametr ovlivnujici rychlost sledovani cary
		 */
		public void startFollowing(float p){
			startFollowing(p,0,0);
		}

		/**
		 * pokud nevis, co delas, radeji pouzij funkci bez parametru i a d
		 * @param p
		 * @param i
		 * @param d
		 */
		public void startFollowing(float p, float i, float d) {
			follow_p = p;
			follow_i = i;
			follow_d = d;
			forward_speed = buggy.motL.getSpeed(); // pripadne *sgn(target_distance)
			print("forward speed=");
			print(forward_speed);
			print("\n");
			sleepMilliseconds(250);
			last_time = elapsedMilliseconds();
			last_e = getOfftrackValue(follow_ls);
			acc_e = 0;
		}

		/**
		 * Dat robota napravo od cary mirit mirne sikmo na caru. Zavolat go(x,true)
		 * a zavolat findTrack(). Funkce se vrati jakmile najede na cernou. Pro naslednou jizdu
		 * po care je pak
		 * treba zavolat startFollowing a volat dokola follow(), kdyz se blizime ke
		 * konci tak zavolat endFollowing() a pak zastavit robota.
		 *
		 * @param searchSpeed
		 *            rychlost v mm/s kterou bude robot hledat trasu
		 */
		public void findTrack(float searchSpeed) {
			buggy.speed(searchSpeed);
			while (getOfftrackValue(follow_ls) > -.3) {
				yield();
			}
		}

		/** jede po cerne care, na jejim pravem okraji */
		public void follow() {
			int now = elapsedMilliseconds();
			if (now - last_time < 10) {
				sleepMilliseconds(12 - (now - last_time));
				now = elapsedMilliseconds();
			}
			float dt = 0.001f * (now - last_time);
			float e = getOfftrackValue(follow_ls); /* 1=bila -1=cerna, cil je 0 */
			if (dt > 0.1f) {
				beep(); // chyba -- musite to volat aspon 10* za sekundu
			} else {
				acc_e = acc_e + follow_i * e * dt;
				if (acc_e >= 1)
					acc_e = 1;
				if (acc_e <= -1)
					acc_e = -1;
				float diff = follow_d * (e - last_e) / dt;
				float a = e * follow_p + acc_e
						+ diff; /* a=relativni sila zataceni vlevo */
				if (a > 1)
					a = 1;
				if (a < -1)
					a = -1;
				float ls = (180 / 3.141592653589f) * (forward_speed * (1 - a * buggy.way) / buggy._R);
				float rs = (180 / 3.141592653589f) * (forward_speed * (1 + a * buggy.way) / buggy._R);
				if (ls > 450)
					ls = 450;
				if (rs > 450)
					rs = 450;
				if (ls <= 5)
					ls = 5;
				if (rs <= 5)
					rs = 5;
				print("a=" + a + " acce=" + acc_e + " \r");
				buggy.motR.origsetSpeed((int) rs);
				buggy.motL.origsetSpeed((int) ls);
			}
			last_time = now;
			last_e = e;
			yield();
		}

		public void stopFollowing() {
			buggy.speed(forward_speed);
		}

	}


	private static float sgn(float x) {
		if (x > 0)
			return 1;
		else
			return -1;
	}

	public static class Buggy {

		private NXTRegMotor motL, motR;
		private float _R, _L;
		private boolean reverseLeft;
		private boolean reverseRight;

		public Buggy(float prumer, float rozchod, NXTRegMotor levyMotor, NXTRegMotor pravyMotor, boolean reverseLeft,
				boolean reverseRight) {
			motL = levyMotor;
			motR = pravyMotor;
			_R = prumer * 0.5f;
			_L = rozchod * 0.5f;
			this.reverseLeft = reverseLeft;
			this.reverseRight = reverseRight;
			speed(1000);
		}

		private float way; /* 1 vpred (mm>0), -1 vzad (mm<0) */
		private float target_dist;
		private int leftAngularSpeed;
		private int rightAngularSpeed;

		/**
		 * jede vpred mm milimietru
		 *
		 * @param millimeters
		 *            o kolik milimetru mam popojet
		 * @param nonBlocking
		 *            true: zadej ukol a hned vyzkoc z volani prikazu, false:
		 *            pockej, dokud se nedojede zadana vzdalenost
		 */
		public void go(float millimeters, boolean nonBlocking) {
			target_dist = Math.abs(millimeters);
			way = (int) sgn(millimeters);
			motR.resetTachoCount();
			motL.resetTachoCount();
			float alfa = (180 / 3.141592653589f) * millimeters / _R;
			int ialfa = Math.round(alfa); /* zaokrouhleni k nejblizsimu cislu */
			int leftAngle = reverseLeft ? -ialfa : ialfa;
			int rightAngle = reverseRight ? -ialfa : ialfa;
			motR.rotate(rightAngle, true);
			motL.rotate(leftAngle, nonBlocking);
		}

		/**
		 * vrati absolutni hodnotu aktualni ujetou vzdalenost v mm od posledniho
		 * zavolani go() nebo turn() Vraci vzdy kladne cislo, i kdyz se couvalo
		 * napr. pomoci go(-100);
		 */
		public float getCurrentDistance() {
			int double_angle = motL.getTachoCount() + motR.getTachoCount();
			return Math.abs((3.141592653589f / 360) * double_angle * _R);
		}

		/**
		 * Popojede o zadanou vzdalenost
		 *
		 * @param mm
		 *            o kolik popojet
		 */
		public void go(float mm) {
			go(mm, false);
		}

		/**
		 * Vrati, zda se robot hybe nebo ne
		 *
		 * @return true: pokud se hybe, jinak false
		 */
		public boolean isGoing() {
			return motL.isMoving() || motR.isMoving();
		}

		/**
		 * Zastavit. Uprimne uplne nevim, co to dela.
		 *
		 * @param brake_dist
		 * @param discard_dist
		 */
		public void brake(float brake_dist, float discard_dist) {
			float brake_milestone = target_dist - (brake_dist + discard_dist);
			float discard_milestone = target_dist - discard_dist;
			if (brake_milestone > 0) {
				while (getCurrentDistance() < brake_milestone) {
					yield();
				}
			}
			while (true) {
				float where_am_i = getCurrentDistance();
				speed(5 + forward_speed * (discard_milestone - where_am_i) / brake_dist);
				if (where_am_i >= discard_milestone) {
					motL.stop();
					motR.stop();
					break;
				}
				sleepMilliseconds(100);
			}
			speed(forward_speed);
		}

		/**
		 * otoci se vlevo o deg stupnu (zaporny deg toci pochopitelne vpravo)
		 *
		 * @param deg
		 *            uhel ve stupnich, o ktery se otocit
		 * @param non_blocking
		 *            true: vrat hned jak zadas ukol, false: pockej, nez se
		 *            dokonci
		 */
		public void turn(float deg, boolean non_blocking) {
			motR.resetTachoCount();
			motL.resetTachoCount();
			/*
			 * float mm = (3.141592653589f*deg/180)*_L; float
			 * alfa=(180/3.141592653589f)*mm/_R;
			 */
			float alfa = deg * _L
					/ _R; /* to je totez, ale da to min prace spocitat */
			int ialfa = Math.round(alfa);
			int leftAngle = reverseLeft ? -ialfa : ialfa;
			int rightAngle = reverseRight ? -ialfa : ialfa;

			motR.rotate(rightAngle, true);
			motL.rotate(-leftAngle, non_blocking);
		}

		/**
		 * Otoci se vlevo o deg stupnu, pocka, nez se dootaci
		 *
		 * @param deg
		 *            stupnu o ktere se otocit
		 */
		public void turn(float deg) {
			turn(deg, false);
		}

		/**
		 * Zacne se otacet vlevo.
		 */
		public void turn() {
			int speedLeft = reverseLeft?-leftAngularSpeed:leftAngularSpeed;
			int speedRight = reverseRight?-rightAngularSpeed:rightAngularSpeed;
			motL.setSpeed(speedLeft);
			motR.setSpeed(-speedRight);
		}


		/**
		 * Rozjede se dopredu
		 */
		public void forward() {
			int speedLeft = reverseLeft?-leftAngularSpeed:leftAngularSpeed;
			int speedRight = reverseRight?-rightAngularSpeed:rightAngularSpeed;
			motL.setSpeed(speedLeft);
			motR.setSpeed(speedRight);
		}

		/**
		 * Rozjede se dozadu
		 */
		public void backward() {
			int speedLeft = reverseLeft?-leftAngularSpeed:leftAngularSpeed;
			int speedRight = reverseRight?-rightAngularSpeed:rightAngularSpeed;
			motL.setSpeed(-speedLeft);
			motR.setSpeed(-speedRight);
		}

		/**
		 * Zastavi robota brzdenim
		 */
		public void stop() {
			motL.setSpeed(0);
			motR.setSpeed(0);
		}

		/**
		 * nastavi rychlost v mm/s
		 */
		public void speed(float mmps) {
			speed(mmps, mmps);
		}

		/**
		 * nastavi rychlost v mm/s zvlast pro leve a prave kolo
		 */
		public void speed(float left_mmps, float right_mmps) {
			leftAngularSpeed = mmpsTodps(left_mmps);
			if (leftAngularSpeed > 450)
				leftAngularSpeed = 450;
			motL.origsetSpeed(leftAngularSpeed);
			rightAngularSpeed = mmpsTodps(right_mmps);
			if (rightAngularSpeed > 450)
				rightAngularSpeed = 450;
			motR.origsetSpeed(rightAngularSpeed);
		}

		private int mmpsTodps(float right_mmps) {
			return (int)((180 / 3.141592653589f) * right_mmps / _R);
		}

		/**
		 * nastavi se maximalni zrychleni robota
		 *
		 * @param mmpss
		 *            zrychleni, ktere se muze pouzit
		 */
		public void acceleration(float mmpss) // maximalni zrychleni robota v
												// mm/s/s
		{
			int a = Math.round((180 / 3.141592653589f) * mmpss / _R);
			motR.setAcceleration(a);
			motL.setAcceleration(a);
		}
	}
}
