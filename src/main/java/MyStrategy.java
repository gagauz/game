import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;
import model.Car;
import model.Direction;
import model.Game;
import model.Move;
import model.TileType;
import model.World;

public final class MyStrategy implements Strategy {

    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_90 = PI / 2;
    private static final double ANGLE_89 = ANGLE_90 / 90 * 89;
    private static final double ANGLE_45 = PI / 4;

    private Car self;
    private World world;
    private Game game;
    private Move move;

    boolean enableCenter = false;

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
    private TileType lastTile = TileType.EMPTY;
    private TileType nextTile = TileType.EMPTY;

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

        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;
        this.nextTile = getNext();

        //        for (int x = 0; x < world.getTilesXY().length; x++) {
        //            for (int y = 0; y < world.getTilesXY()[x].length; y++) {
        //                System.out.println(world.getTilesXY()[x][y].name());
        //            }
        //        }

        if (world.getTick() > game.getInitialFreezeDurationTicks()) {

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
        if ((deltaSpeed() < 0.0001 && deltaMove() < 0.0001) && tickTr > 20) {
            System.exit(1);
            return true;
        }
        return false;
    }

    void checkTurn() {
        if (nextTile != lastTile) {
            System.out.println("" + lastTile + " -> " + nextTile);
            if (nextTile == TileType.LEFT_TOP_CORNER) {
                if (lastTile == TileType.VERTICAL && self.getSpeedY() < 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() < 0) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == TileType.RIGHT_TOP_CORNER) {
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() > 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.VERTICAL && self.getSpeedY() < 0) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == TileType.RIGHT_BOTTOM_CORNER) {
                if (lastTile == TileType.VERTICAL && self.getSpeedY() > 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() > 0) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == TileType.LEFT_BOTTOM_CORNER) {
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() < 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.VERTICAL && self.getSpeedY() > 0) {
                    delegate = new TurnLeft();
                }
            }

            lastTile = nextTile;
        }
    }

    double speed() {
        return sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
    }

    boolean checkClose() {
        for (Car car : world.getCars()) {
            if (self.getId() != car.getId()) {
                if (self.getDistanceTo(car) < self.getHeight()) {
                    System.out.println("Close!! " + self.getDistanceTo(car) + " " + self.getHeight());
                    return true;
                }
            }
        }
        return false;
    }

    void checkStraight() {
        double a = correct(self.getAngle());
        double d = a;
        if (nextTile == TileType.HORIZONTAL) {
            d = between(a, -ANGLE_90, ANGLE_90) ? 0 : ANGLE_180;
        } else if (nextTile == TileType.VERTICAL) {
            d = between(a, 0, ANGLE_180) ? ANGLE_90 : -ANGLE_90;
        }
        double c = correct(d - a);
        if (c != 0) {
            System.out.println("Move wheel to " + c + ", a=" + a + ", d=" + d);
            move.setWheelTurn(2 * c);
        }
    }

    void checkCenter() {
        /*
        if (!checkClose()) {
            System.out.println("Center");
            double w2 = game.getTrackTileSize() / 2;
            int inTileX = ((int) self.getX()) % ((int) game.getTrackTileSize());
            int inTileY = ((int) self.getY()) % ((int) game.getTrackTileSize());
            double s = speed() * game.getTrackTileSize();
            if (lastTile == TileType.HORIZONTAL) {
                double d = w2 - inTileY;
                if (abs(d) > 0) {
                    move.setWheelTurn(signum(d) * 0.3);
                } else {
                    move.setWheelTurn(0);
                }
            }
            if (lastTile == TileType.VERTICAL) {
                double d = w2 - inTileX;
                if (abs(d) > 0) {
                    move.setWheelTurn(signum(d) * 0.3);
                } else {
                    move.setWheelTurn(0);
                }
            }
        }
        */
    }

    TileType getNext() {
        int x = self.getNextWaypointX();
        int y = self.getNextWaypointY();
        return world.getTilesXY()[x][y];
    }

    void print() {
        int x = self.getNextWaypointX();
        int y = self.getNextWaypointY();
        System.out.println(world.getTilesXY()[x][y]);
    }

    class MoveForward implements Delegate {
        MoveForward() {
            print();
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            System.out.println("Begin move forward");
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkTurn();
            checkStraight();
            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                print();
                delegate = new MoveBackward(new TurnRight());
            }
        }
    }

    double deltaBack() {
        return sqrt(pow(moveBackBeginX - self.getX(), 2) + pow(moveBackBeginY - self.getY(), 2));
    }

    class MoveBackward implements Delegate {
        Turn next;

        MoveBackward(Turn next) {
            tickTr = 0;
            this.next = next;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            System.out.println("Begin move backward");
        }

        @Override
        public void move() {
            move.setWheelTurn(-next.get());
            if (deltaBack() < self.getHeight()) {
                move.setEnginePower(-1.0D);
            } else if (deltaBack() < self.getHeight() * 2) {
                move.setEnginePower(1.0D);
            } else {
                move.setEnginePower(0.0D);
                move.setBrake(true);
            }
            if (move.getEnginePower() == 0 && deltaSpeed() < 0.0001) {
                System.out.println("Start delegate " + next);
                tickTr = 0;
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

        double needAngle;

        Turn() {
            tickTr = 0;
            turnAngleBegin = closestAngle(self.getAngle());
            needAngle = turnAngleBegin + get() * ANGLE_90;
            System.out.println("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + needAngle);
        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);

            double delta = self.getAngle() - turnAngleBegin;
            if (abs(delta) >= ANGLE_89) {
                System.out.println("Angle is " + self.getAngle() + ", delta is " + delta + " stop turn");
                nextTile = getNext();
                delegate = new MoveForward();
            } else {
                move.setWheelTurn(get());
            }

            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                print();
                delegate = new MoveBackward(new TurnRight());
            }
        }
    }

    public double closestAngle(double angle) {
        if (between(angle, -ANGLE_45, ANGLE_45)) {
            return 0;
        }
        if (between(angle, ANGLE_45, ANGLE_90 + ANGLE_45)) {
            return ANGLE_90;
        }
        if (between(angle, -ANGLE_180 + ANGLE_45, -ANGLE_45)) {
            return -ANGLE_90;
        }
        return ANGLE_180;
    }

    double correctDelta(double a1, double a2) {
        if (a1 < 0)
            a1 = a1 + ANGLE_360;
        if (a2 < 0)
            a2 = a2 + ANGLE_360;
        return a1 - a2;
    }

    double correct(double a) {
        if (a > ANGLE_180) {
            return a - ANGLE_360;
        }
        return a;
    }

    boolean between(double v1, double v2, double v3) {
        return v1 > v2 && v1 < v3;
    }
}
