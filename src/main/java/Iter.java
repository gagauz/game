import java.io.File;
import java.io.IOException;

public class Iter {
    public static void main(String[] args) throws IOException, InterruptedException {
        File dir = new File("R:\\projects-my\\game\\local-runner\\");
        double a = Double.parseDouble(args[0]);// -0.5;
        double a1 = Double.parseDouble(args[1]);//  2;
        double da = 0.1;

        double b = Double.parseDouble(args[3]);//  -5;
        double b1 = Double.parseDouble(args[4]);//  5;
        double db = 1;

        double c = Double.parseDouble(args[6]);//  -2000;
        double c1 = Double.parseDouble(args[7]);//  2000;
        double dc = 100;

        double d = Double.parseDouble(args[9]);//  0.5;
        double d1 = Double.parseDouble(args[10]);//  1;
        double dd = 0.1;

        double e = Double.parseDouble(args[12]);//  0.5;
        double e1 = Double.parseDouble(args[13]);//  1;
        double de = 0.1;

        double f = Double.parseDouble(args[15]);// 12;
        double f1 = Double.parseDouble(args[16]);// 30;
        double df = 1;
        for (; d <= d1; d += dd) {
            for (; e <= e1; e += de) {
                for (; f <= f1; f += df) {
                    for (; a <= a1; a += da) {
                        for (; b <= b1; b += db) {
                            for (; c <= c1; c += dc) {
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
