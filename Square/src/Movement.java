
public class Movement extends Robotabor {

	public static void main(String[] args) {
		//int y = 1;
		init();
		initBuggy(56, 110);
     	//while(y>0){
     		jezdiDoCtverce();
     	//}
	}

	private static void jezdiDoCtverce() {
		//int i = 4;
		//while (i >0){
			buggy.go(200);
			buggy.turn(90);
			//i--
			jezdiDoCtverce(); //pryc
		//}

	}

}
