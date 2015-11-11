import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;
import static model.Direction.DOWN;
import static model.Direction.LEFT;
import static model.Direction.RIGHT;
import static model.Direction.UP;
import static model.TileType.BOTTOM_HEADED_T;
import static model.TileType.EMPTY;
import static model.TileType.LEFT_BOTTOM_CORNER;
import static model.TileType.LEFT_HEADED_T;
import static model.TileType.LEFT_TOP_CORNER;
import static model.TileType.RIGHT_BOTTOM_CORNER;
import static model.TileType.RIGHT_HEADED_T;
import static model.TileType.RIGHT_TOP_CORNER;
import static model.TileType.TOP_HEADED_T;
import model.Car;
import model.Direction;
import model.Game;
import model.Move;
import model.TileType;
import model.World;

public final class MyStrategy implements Strategy {

    static double turnMaxSpeed = 10;
    static double straightFactor = 0.0010;

    private static final double[] TURN_OFFSET0 = new double[] {1, -1, -1, 1};
    private static final double[] DIR = new double[] {-1, 1, -1, 1};
    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_90 = PI / 2;
    private static final double ANGLE_89 = ANGLE_90 / 90 * 89;
    private static final double ANGLE_45 = PI / 4;

    private static int tileSize;

    private Car self;
    private World world;
    private Game game;
    private Move move;

    boolean enableCenter = false;
    boolean turning = false;

    private double lastX;
    private double lastY;
    private double lastSpeedX;
    private double lastSpeedY;
    private double lastAngle;
    private double turnAngleBegin;
    private double nextTurnX;
    private double nextTurnY;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;

    private int cx;
    private int cy;
    private Direction direction;

    private int tickTr = 0;

    private Delegate delegate;
    private TileType lastTile = EMPTY;
    private TileType nextTile = EMPTY;

    //    

    @Override
    public void move(Car self, World world, Game game, Move move) {

        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;

        if (null == direction) {
            direction = world.getStartingDirection();
            tileSize = (int) game.getTrackTileSize();
        }
        cx = (int) self.getX() / tileSize;
        cy = (int) self.getY() / tileSize;

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

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

        nextTile = getNextTile();

        if (nextTile != lastTile) {
            System.out.println("Turn: " + lastTile + " -> " + nextTile);
            if (nextTile == LEFT_TOP_CORNER) {
                if (isUp()) {
                    delegate = new TurnRight();
                }
                if (isLeft()) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == RIGHT_TOP_CORNER) {
                if (isRight()) {
                    delegate = new TurnRight();
                }
                if (isUp()) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == RIGHT_BOTTOM_CORNER) {
                if (isDown()) {
                    delegate = new TurnRight();
                }
                if (isRight()) {
                    delegate = new TurnLeft();
                }
            }

            if (nextTile == LEFT_BOTTOM_CORNER) {
                if (isLeft()) {
                    delegate = new TurnRight();
                }
                if (isDown()) {
                    delegate = new TurnLeft();
                }
            }

            lastTile = nextTile;
        }
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
        if (!turning) {
            double need = getRightAngle();
            double real = self.getAngle();

            double c = abs(real) - abs(need);
            double offset = getCenterOffset(0, 0);
            //            System.out.println("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);
            double da = straightFactor * TURN_OFFSET0[direction.ordinal()] * offset;
            System.out.println("Dir o1=" + -2 * c * DIR[direction.ordinal()] + ", o2=" + da);
            move.setWheelTurn(-4 * c * DIR[direction.ordinal()]);
        }
    }

    double getCenterOffset(double x, double y) {
        if (0 >= x)
            x = cx * tileSize + tileSize / 2;
        if (0 >= y)
            y = cy * tileSize + tileSize / 2;
        double offset = isHorizontal()
                ? self.getY() - y
                : self.getX() - x;
        return offset;
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
            checkBrake();
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

    void checkBrake() {
        boolean brakeOn = false;
        /*
        if (turning || getDistToNextTurn() < tileSize)
            brakeOn = speed > turnMaxSpeed;
        if (brakeOn) {
            System.out.println("Bake ON! " + speed);
            move.setEnginePower(0.5);
            move.setBrake(true);
        }
        */
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
            turnAngleBegin = getRightAngle();
            needAngle = turnAngleBegin + get() * ANGLE_90;
            System.out.println("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + needAngle);

            turnBegin(this);
        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkBrake();

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));
            if (delta >= ANGLE_89) {
                System.out.println("Angle is " + self.getAngle() + ", delta is " + delta + " stop turn");
                turnEnd(this);
            } else {
                move.setWheelTurn(get());
            }

            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                print();
                delegate = new MoveBackward(new TurnRight());
            }
        }

        boolean isLeft() {
            return get() < 0;
        }

        boolean isRight() {
            return get() > 0;
        }
    }

    public double getRightAngle() {
        switch (direction) {
        case UP:
            return -ANGLE_90;
        case DOWN:
            return ANGLE_90;
        case LEFT:
            return ANGLE_180;
        default:
            return 0;
        }
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

    TileType getTile() {
        return world.getTilesXY()[cx][cy];
    }

    TileType getNextTile() {
        int x = cx;
        int y = cy;
        if (isUp() && cy > 0)
            y--;
        else if (isDown() && cy < world.getHeight())
            y++;
        else if (isLeft() && cx > 0)
            x--;
        else if (isRight() && cx < world.getWidth())
            x++;
        return world.getTilesXY()[x][y];
    }

    boolean isHorizontal() {
        return direction == LEFT || direction == RIGHT;
    }

    boolean isVertical() {
        return direction == UP || direction == DOWN;
    }

    boolean isLeft() {
        return direction == LEFT;
    }

    boolean isRight() {
        return direction == RIGHT;
    }

    boolean isUp() {
        return direction == UP;
    }

    boolean isDown() {
        return direction == DOWN;
    }

    interface Flow {
        boolean apply(TileType tileType, int cx, int i, int c);
    }

    void flow(Flow flow) {
        int d = isDown() || isRight() ? 1 : -1;
        if (isVertical()) {
            for (int i = cy, c = 0; i < world.getHeight() && i > -1; i = i + d, c++) {
                if (!flow.apply(world.getTilesXY()[cx][i], cx, i, c)) {
                    break;
                }
            }
        } else {
            for (int i = cx, c = 0; i < world.getWidth() && i > -1; i = i + d, c++) {
                if (!flow.apply(world.getTilesXY()[i][cy], i, cy, c)) {
                    break;
                }
            }
        }
    }

    private final Flow nextTurnFinder = new Flow() {

        @Override
        public boolean apply(TileType tileType, int x, int y, int c) {
            if (c > 0 && isTurnTile(tileType)) {
                nextTurnX = tileSize * x + tileSize / 2;
                nextTurnY = tileSize * y + tileSize / 2;
                return false;
            }
            return true;
        }

    };

    boolean isTurnTile(TileType type) {
        boolean turn = (LEFT_TOP_CORNER == type)
                || (LEFT_BOTTOM_CORNER == type)
                || (RIGHT_BOTTOM_CORNER == type)
                || (RIGHT_TOP_CORNER == type);
        if (!turn) {
            turn = (LEFT_HEADED_T == type && isRight())
                    || (RIGHT_HEADED_T == type && isLeft())
                    || (TOP_HEADED_T == type && isDown())
                    || (BOTTOM_HEADED_T == type && isUp());
        }
        return turn;
    }

    void turnBegin(Turn turn) {
        System.out.println("Turn begin " + direction);
        turning = true;
    }

    void turnEnd(Turn turn) {
        if (isUp())
            direction = turn.isLeft() ? LEFT : RIGHT;
        else if (isDown())
            direction = turn.isLeft() ? RIGHT : LEFT;
        else if (isLeft())
            direction = turn.isLeft() ? DOWN : UP;
        else if (isRight())
            direction = turn.isLeft() ? UP : DOWN;

        System.out.println("Turn end " + direction);
        turning = false;
        flow(nextTurnFinder);
        delegate = new MoveForward();

    }

    double getEndSpeed(double dist) {
        if (0 == accel)
            return speed;
        return sqrt(speed * speed + 2 * dist * accel);
    }

    double getDistToNextTurn() {
        return self.getDistanceTo(nextTurnX, nextTurnY);
    }
}
