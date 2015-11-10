import model.*;

import static java.lang.Math.*;

public final class MyStrategy implements Strategy {

    private static final double ANGLE_90 = 1.5708;
    private static final double ANGLE_45 = 1.5708 / 2;

    private static Car self;
    private static World world;
    private static Game game;
    private static Move move;

    private double lastX;
    private double lastY;
    private double lastSpeedX;
    private double lastSpeedY;
    private double lastAngle;
    private double turnAngleBegin;
    private double moveBackBeginX;
    private double moveBackBeginY;

    private int tickTr = 0;

    private Delegate delegate;

    Direction getDirection() {
        if (self.getAngle() >= -ANGLE_45 && self.getAngle() < ANGLE_45) {
            return Direction.RIGHT;
        }
        if (self.getAngle() >= ANGLE_45 && self.getAngle() < ANGLE_45 + ANGLE_90) {
            return Direction.UP;
        }
        if (self.getAngle() >= ANGLE_45 + ANGLE_90 && self.getAngle() < ANGLE_45 + ANGLE_90 + ANGLE_90) {
            return Direction.LEFT;
        }
        return Direction.DOWN;
    }

    //    

    @Override
    public void move(Car self, World world, Game game, Move move) {

        MyStrategy.self = self;
        MyStrategy.world = world;
        MyStrategy.game = game;
        MyStrategy.move = move;

        //        for (int x = 0; x < world.getTilesXY().length; x++) {
        //            for (int y = 0; y < world.getTilesXY()[x].length; y++) {
        //                System.out.println(world.getTilesXY()[x][y].name());
        //            }
        //        }

        if (world.getTick() > game.getInitialFreezeDurationTicks()) {

            int x = self.getNextWaypointX();
            int y = self.getNextWaypointY();
            System.out.println(world.getTilesXY()[x][y]);

            tickTr++;
            if (null != delegate) {
                delegate.move();
            } else {
                delegate = new MoveForward();
            }
        }

        lastX = self.getX();
        lastY = self.getY();

        lastSpeedX = self.getSpeedX();
        lastSpeedY = self.getSpeedY();
        lastAngle = self.getAngle();
        //        System.out.println("X=" + self.getX() + ", Y=" + self.getY() + ", A=" + self.getAngle() + ", Vx=" + self.getSpeedX() + ", Vy=" + self.getSpeedY());

    }

    double deltaSpeed() {
        return sqrt(pow(self.getSpeedX() - lastSpeedX, 2) + pow(self.getSpeedY() - lastSpeedY, 2));
    }

    long deltaMove() {
        return round(sqrt(pow(self.getX() - lastX, 2) + pow(self.getY() - lastY, 2)));
    }

    interface Delegate {
        void move();
    }

    void detectStale() {
        if (isStale()) {
        }
    }

    boolean isStale() {
        return (deltaSpeed() < 0.001 && deltaMove() < 0.001) && tickTr > 50;
    }

    class MoveForward implements Delegate {
        MoveForward() {
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            System.out.println("Begin move forward");
        }

        @Override
        public void move() {
            move.setEnginePower(0.5D);
            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                delegate = new MoveBackward(new TurnRight());
            }
        }
    }

    class MoveBackward implements Delegate {
        Delegate next;

        MoveBackward(Delegate next) {
            tickTr = 0;
            this.next = next;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            System.out.println("Begin move backward");
        }

        @Override
        public void move() {
            move.setEnginePower(-1.0D);
            if (abs(moveBackBeginX - self.getX()) > self.getHeight() || abs(moveBackBeginY - self.getY()) > self.getHeight()) {
                System.out.println("Stop move backward");
                System.out.println("Start delegate " + next);
                move.setEnginePower(1.0);
                delegate = next;
            }
        }
    }

    class TurnLeft extends Turn {

        @Override
        double get() {
            return -1.0;
        }
    }

    class TurnRight extends Turn {

        @Override
        double get() {
            return 1.0;
        }
    }

    abstract class Turn implements Delegate {
        Turn() {
            tickTr = 0;
            turnAngleBegin = self.getAngle();
        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);
            move.setWheelTurn(get());
            if (abs(self.getAngle() - turnAngleBegin) >= ANGLE_90) {
                System.out.println("Angle id " + (self.getAngle() - turnAngleBegin) + " stop turn");
                delegate = new MoveForward();
            }
        }
    }

}
