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
        float a0 = Float.parseFloat(args[1]);// -0.5;
        float a1 = Float.parseFloat(args[2]);//  2;
        float da = 0.5f;

        float b0 = Float.parseFloat(args[3]);//  -5;
        float b1 = Float.parseFloat(args[4]);//  5;
        float db = 1;

        float c0 = Float.parseFloat(args[5]);//  -2000;
        float c1 = Float.parseFloat(args[6]);//  2000;
        float dc = 100f;

        float d0 = Float.parseFloat(args[7]);//  0.5;
        float d1 = Float.parseFloat(args[8]);//  1;
        float dd = 0.1f;

        float e0 = Float.parseFloat(args[9]);//  0.5;
        float e1 = Float.parseFloat(args[10]);//  1;
        float de = 0.1f;

        float f0 = Float.parseFloat(args[11]);// 12;
        float f1 = Float.parseFloat(args[12]);// 30;
        float df = 1;

        if (null != l) {
            String[] ss = l.split("\t");
            a0 = Float.parseFloat(ss[0]);
            b0 = Float.parseFloat(ss[1]);
            c0 = Float.parseFloat(ss[2]);
            d0 = Float.parseFloat(ss[3]);
            e0 = Float.parseFloat(ss[4]);
            f0 = Float.parseFloat(ss[5]);
        }

        System.out.println(a1 +
                " " + b1 +
                " " + c1 +
                " " + d1 +
                " " + e1 +
                " " + f1);
        for (float d = d0; d <= d1; d += dd) {
            for (float e = e0; e <= e1; e += de) {
                for (float f = f0; f <= f1; f += df) {
                    for (float a = a0; a <= a1; a += da) {
                        for (float b = b0; b <= b1; b += db) {
                            for (float c = c0; c <= c1; c += dc) {
                                float x = f * f * a + f * b + c;
                                if (x < 0 || x > 4000)
                                    continue;
                                x = f1 * f1 * a + f1 * b + c;
                                if (x < 0 || x > 4000)
                                    continue;
                                long t = System.currentTimeMillis();
                                System.out.println("c=" + c);
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
                                    System.out.println("Time: " + (System.currentTimeMillis() - t) + " ms");
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
