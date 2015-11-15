import java.io.File;
import java.io.IOException;

public class Iter {
    public static void main(String[] args) throws IOException, InterruptedException {
        File dir = new File("R:\\projects-my\\game\\local-runner\\");
        double a = -0.5;
        double a1 = 2;
        double da = 0.1;

        double b = -5;
        double b1 = 5;
        double db = 1;

        double c = -2000;
        double c1 = 2000;
        double dc = 100;

        double d = 0.5;
        double d1 = 1;
        double dd = 0.1;

        double e = 0.5;
        double e1 = 1;
        double de = 0.1;

        double f = 12;
        double f1 = 30;
        double df = 2;

        for (; a <= a1; a += da) {
            for (; b <= b1; b += db) {
                for (; c <= c1; c += dc) {
                    for (; d <= d1; d += dd) {
                        for (; e <= e1; e += de) {
                            for (; f <= f1; f += df) {
                                String s = "R:\\projects-my\\game\\local-runner\\local-runner-console.bat" +
                                        " " + a +
                                        " " + b +
                                        " " + c +
                                        " " + d +
                                        " " + e +
                                        " " + f;
                                Process p = Runtime.getRuntime().exec(s, null, dir);
                                p.waitFor();
                            }
                        }
                    }
                }
            }
        }
    }
}
