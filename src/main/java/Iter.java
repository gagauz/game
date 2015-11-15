import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Iter {
    public static void main(String[] args) throws IOException, InterruptedException {

        File out = new File(args[0] + "/log.csv");
        FileReader fr = new FileReader(out);
        BufferedReader fs = new BufferedReader(fr);
        String line;
        String l = null;
        try {
            while ((line = fs.readLine()) != null) {
                System.out.println(line);
                l = line;
            }
            fs.close();
            fr.close();

        } catch (Exception e) {
            e.printStackTrace();
        }

        File dir = new File(args[0]);
        double a = Double.parseDouble(args[1]);// -0.5;
        double a1 = Double.parseDouble(args[2]);//  2;
        double da = 0.1;

        double b = Double.parseDouble(args[3]);//  -5;
        double b1 = Double.parseDouble(args[4]);//  5;
        double db = 1;

        double c = Double.parseDouble(args[5]);//  -2000;
        double c1 = Double.parseDouble(args[6]);//  2000;
        double dc = 100;

        double d = Double.parseDouble(args[7]);//  0.5;
        double d1 = Double.parseDouble(args[8]);//  1;
        double dd = 0.1;

        double e = Double.parseDouble(args[9]);//  0.5;
        double e1 = Double.parseDouble(args[10]);//  1;
        double de = 0.1;

        double f = Double.parseDouble(args[11]);// 12;
        double f1 = Double.parseDouble(args[12]);// 30;
        double df = 1;

        if (null != l) {
            String[] ss = l.split("\t");
            a = Double.parseDouble(ss[0]);
            b = Double.parseDouble(ss[1]);
            c = Double.parseDouble(ss[2]);
            d = Double.parseDouble(ss[3]);
            e = Double.parseDouble(ss[4]);
            f = Double.parseDouble(ss[5]);
        }

        for (; d <= d1; d += dd) {
            for (; e <= e1; e += de) {
                for (; f <= f1; f += df) {
                    for (; a <= a1; a += da) {
                        for (; b <= b1; b += db) {
                            for (; c <= c1; c += dc) {
                                String s = "local-runner-console.bat" +
                                        " " + a +
                                        " " + b +
                                        " " + c +
                                        " " + d +
                                        " " + e +
                                        " " + f;
                                System.out.println(s);
                                try {
                                    Process p = Runtime.getRuntime().exec(s, null, dir);
                                    p.waitFor();
                                } catch (Exception ex) {
                                    ex.printStackTrace();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
