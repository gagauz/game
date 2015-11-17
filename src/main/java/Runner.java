import model.Car;
import model.Game;
import model.Move;
import model.PlayerContext;

import java.io.IOException;

public final class Runner {
    private final RemoteProcessClient remoteProcessClient;
    private final String token;

    private static MyStrategy strategy;

    public static void main(String[] args) throws IOException {
        if (args.length > 4) {
            double a = Double.parseDouble(args[0]);
            double b = Double.parseDouble(args[1]);
            double c = Double.parseDouble(args[2]);
            double d = Double.parseDouble(args[3]);
            double e = Double.parseDouble(args[4]);
            //            double f = Double.parseDouble(args[5]);
            //            double g = Double.parseDouble(args[6]);
            strategy = new MyStrategy(a, b, c, d, e);
        } else {
            strategy = new MyStrategy();
        }
        //        if (args.length == 3) {
        //            new Runner(args).run();
        //        } else {
        //            new Runner(new String[] {"127.0.0.1", "31001", "0000000000000000"}).run();
        //        }
        new Runner(new String[] {"127.0.0.1", "31002", "0000000000000000"}).run();
    }

    private Runner(String[] args) throws IOException {
        remoteProcessClient = new RemoteProcessClient(args[0], Integer.parseInt(args[1]));
        token = args[2];
    }

    public void run() throws IOException {
        try {
            remoteProcessClient.writeToken(token);
            int teamSize = remoteProcessClient.readTeamSize();
            remoteProcessClient.writeProtocolVersion();
            Game game = remoteProcessClient.readGameContext();

            Strategy[] strategies = new Strategy[teamSize];

            for (int strategyIndex = 0; strategyIndex < teamSize; ++strategyIndex) {
                strategies[strategyIndex] = strategy;
            }

            PlayerContext playerContext;

            while ((playerContext = remoteProcessClient.readPlayerContext()) != null) {
                Car[] playerCars = playerContext.getCars();
                if (playerCars == null || playerCars.length != teamSize) {
                    break;
                }

                Move[] moves = new Move[teamSize];

                for (int carIndex = 0; carIndex < teamSize; ++carIndex) {
                    Car playerCar = playerCars[carIndex];

                    Move move = new Move();
                    moves[carIndex] = move;
                    try {
                        strategies[playerCar.getTeammateIndex()].move(
                                playerCar, playerContext.getWorld(), game, move
                                );
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                remoteProcessClient.writeMoves(moves);
            }

            System.out.println(strategy.A + "\t" + strategy.B + "\t" + strategy.C + "\t"
                    + strategy.D + "\t" + strategy.E + "\t" + (strategy.speedAvg / strategy.tickCount));
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            remoteProcessClient.close();
        }
    }
}
