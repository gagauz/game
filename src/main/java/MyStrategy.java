import model.*;

import static java.lang.Math.*;
import static model.Direction.*;
import static model.TileType.*;

public final class MyStrategy implements Strategy {

    static double insideTurnFactor = 1.0;//0..1
    static double outsideTurnFactor = 1.0;//0..1
    static double straightFactor = 0.0020;

    static double turnMaxSpeed = 15;
    static boolean canNitro = true;
    static int staleTickCount = 50;
    static double staleSpeedValue = 0.1;
    static double staleMoveValue = 1;
    static double turnRadiusFactor = 1.0;

    private static final double[] OFFSET = new double[] {1, -1, -1, 1};
    private static final double[] DIR = new double[] {-1, 1, -1, 1};

    private static final double[][] OFFS_ANGLE_SIGN = new double[][] {
            //LEFT //RIGHT //UP //DOWN
            /*LEFT  */{0, 0, -1, 1},
            /*RIGHT */{0, 0, 1, -1},
            /*UP    */{1, -1, 0, 0},
            /*DOWN  */{-1, 1, 0, 0}
    };

    private static final double[][] TURN_ANGLE_OUT = new double[][] {
            //LEFT //RIGHT //UP //DOWN
            /*LEFT  */{0, 0, 1, -1},
            /*RIGHT */{0, 0, -1, -1},
            /*UP    */{1, 1, 0, 0},
            /*DOWN  */{-1, 1, 0, 0}
    };
    private static final double[][] TURN_OFFSET_OUT = new double[][] {
            //LEFT //RIGHT //UP //DOWN
            /*LEFT  */{0, 0, 1, -1},
            /*RIGHT */{0, 0, 1, -1},
            /*UP    */{1, -1, 0, 0},
            /*DOWN  */{1, -1, 0, 0}
    };

    private static final double[][] TURN_CENTER_OFFSET_X = new double[][] {
            //        LEFT RIGHT UP DOWN
            /*LEFT  */{0, 0, 1, 1},
            /*RIGHT */{0, 0, -1, -1},
            /*UP    */{-1, 1, 0, 0},
            /*DOWN  */{-1, 1, 0, 0}
    };

    private static final double[][] TURN_CENTER_OFFSET_Y = new double[][] {
            //        LEFT RIGHT UP DOWN
            /*LEFT  */{0, 0, -1, 1},
            /*RIGHT */{0, 0, -1, 1},
            /*UP    */{1, 1, 0, 0},
            /*DOWN  */{-1, -1, 0, 0}
    };

    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_90 = PI / 2;
    private static final double ANGLE_80 = ANGLE_90 / 90 * 80;
    private static final double ANGLE_45 = PI / 4;
    private static final double ANGLE_15 = PI / 12;

    private static int tileSize;

    private Car self;
    private World world;
    private Game game;
    private Move move;

    boolean enableCenter = false;
    boolean turning = false;
    boolean turningHalf = false;

    private double lastX;
    private double lastY;
    private double lastSpeedX;
    private double lastSpeedY;
    private double turnAngleBegin;

    class NextTurn {
        double x;
        double y;
        int tileX;
        int tileY;
        double distance;
        TileType tile = TileType.EMPTY;
        Direction direction = Direction.LEFT;
    }

    private NextTurn currentTurn = new NextTurn();
    private NextTurn nextTurn;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;
    private boolean brakeOn;
    private boolean nitroOn;
    private boolean staleOn;

    private int cx;
    private int cy;
    private double centerX;
    private double centerY;
    private Direction direction;

    private int turnCount = 0;
    private int tickTr = 0;
    private int curentTick = 0;
    private int nitroCountDownStart = 0;
    private int staleMoveDelay;

    private Delegate delegate;
    private TileType lastTile = EMPTY;
    private TileType nextTile = EMPTY;

    //    

    @Override
    public void move(Car self, World world, Game game, Move move) {

        if (0 == tileSize) {
            tileSize = (int) game.getTrackTileSize();
        }
        cx = (int) self.getX() / tileSize;
        cy = (int) self.getY() / tileSize;

        this.curentTick++;
        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;

        if (null == direction) {
            direction = world.getStartingDirection();
            staleMoveDelay = (int) (1.0 / game.getCarWheelTurnChangePerTick());
        }

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

        getNextTurn().distance = self.getDistanceTo(getNextTurn().x, getNextTurn().y);

        defineCenter();

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

        if (nitroOn && nitroCountDownStart < curentTick - game.getNitroDurationTicks()) {
            System.out.println("Nitro off!!");
            nitroOn = false;
        }
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
        //System.out.println(deltaSpeed() + " " + deltaMove() + " " + tickTr + " pwr:" + move.getEnginePower() + " brake:" + move.isBrake());
        if ((deltaMove() < staleMoveValue)
                && (speed < staleSpeedValue)
                && (accel < staleSpeedValue)
                && tickTr > staleTickCount) {
            staleOn = true;
            brakeOn = false;
            move.setBrake(false);
            return true;
        }
        return false;
    }

    void checkTurn() {

        if (staleOn)
            return;

        // TODO Если ускорение - искать раньше 
        if (turning)
            nextTile = getNextTileOf(currentTurn.tileX, currentTurn.tileY, 1);
        else
            nextTile = getNextTile(speed > turnMaxSpeed * 2.5 ? 2 : 1);

        if (nextTile != lastTile) {
            double offs = getCenterOffset() * TURN_ANGLE_OUT[direction.ordinal()][getNextTurn().direction.ordinal()];
            double dist = getNextTurn().distance - tileSize / 2;
            double start = tileSize / 2;
            //Чем ближе к краю и чем меньше скорость, тем больше задержка

            double offsAngle = getOffsetAngle();

            if (offs > 0) {
                //Фактор скорости
                start += speed / turnMaxSpeed * tileSize * speed * (speed > turnMaxSpeed * 2.5 ? 2 : 0.01);
                //Фактор смещения
                start -= offs * 3.0;
                //Фактор угла вхождения в поворот
                start -= offsAngle * OFFS_ANGLE_SIGN[direction.ordinal()][getNextTurn().direction.ordinal()] * 5.0;
            }

            if (cx == currentTurn.tileX && cy == currentTurn.tileY) {
                start = tileSize * 1000;
            }

            if (dist < start) {

                System.out.println("Turn: " + lastTile + " -> " + nextTile);
                Direction dir = resolveDirectionAfterNextTurn(nextTile);
                if (null != dir && dir != direction) {
                    if (isUp()) {
                        if (RIGHT == dir)
                            delegate = new TurnRight();
                        if (LEFT == dir)
                            delegate = new TurnLeft();
                    } else if (isDown()) {
                        if (RIGHT == dir)
                            delegate = new TurnLeft();
                        if (LEFT == dir)
                            delegate = new TurnRight();
                    } else if (isLeft()) {
                        if (UP == dir)
                            delegate = new TurnRight();
                        if (DOWN == dir)
                            delegate = new TurnLeft();
                    } else if (isRight()) {
                        if (DOWN == dir)
                            delegate = new TurnRight();
                        if (UP == dir)
                            delegate = new TurnLeft();
                    }
                }
                lastTile = nextTile;
            }

        }
        if (null == delegate)
            delegate = new MoveForward();
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
            double delta1 = 2 * getOffsetAngle();

            double offset = getCenterOffset();
            //            System.out.println("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

            double delta2 = straightFactor * OFFSET[direction.ordinal()] * offset;
            //            System.out.println("Dir o1=" + delta1 + ", o2=" + delta2);

            delta1 += delta2;
            //            move.setEnginePower(0.2 + abs(ANGLE_90 - abs(offset)) / ANGLE_90);
            move.setWheelTurn(move.getWheelTurn() + delta1);
        }
    }

    void defineCenter() {
        if (!turning) {

            centerX = cx * tileSize + tileSize / 2;
            centerY = cy * tileSize + tileSize / 2;

            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            Bonus b = findNextBonus();
            if (null != b) {
                double dx = b.getX() - centerX;
                double dy = b.getY() - centerY;
                centerX += abs(dx) > self.getWidth() / 2 && abs(dx) < tileSize / 2 ? dx : 0;
                centerY += abs(dy) > self.getWidth() / 2 && abs(dy) < tileSize / 2 ? dy : 0;
            } else if (null != getNextTurn().direction) {
                if (getNextTurn().distance < tileSize * 4 && getNextTurn().distance > tileSize) {
                    double m = getNextTurn().distance < tileSize * 1.2 * speed / turnMaxSpeed ? -insideTurnFactor : outsideTurnFactor;
                    centerX += m * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().direction.ordinal()] * self.getWidth();
                    centerY += m * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().direction.ordinal()] * self.getWidth();
                }
            }
        }
    }

    double getCenterOffset() {

        double offset = isHorizontal()
                ? self.getY() - centerY
                : self.getX() - centerX;
        return offset;
    }

    void print() {
        int x = self.getNextWaypointX();
        int y = self.getNextWaypointY();
        System.out.println(world.getTilesXY()[x][y]);
    }

    class MoveForward implements Delegate {
        MoveForward() {
            staleOn = false;
            turning = false;
            print();
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            System.out.println("Begin move forward");
        }

        @Override
        public void move() {

            move.setEnginePower(1.0 * tickTr / staleTickCount);
            checkBrake();
            checkTurn();
            checkStraight();
            checkNitro();
            checkCanThrowProjectile();
            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                print();
                delegate = new MoveBackward();
            }
        }
    }

    double deltaBack() {
        return sqrt(pow(moveBackBeginX - self.getX(), 2) + pow(moveBackBeginY - self.getY(), 2));
    }

    double getOffsetAngle() {
        double need = getRightAngle();
        double real = self.getAngle();
        if (need == ANGLE_180 && real < 0)
            need = -need;

        double offsAngle = (need - real);
        return offsAngle * OFFS_ANGLE_SIGN[direction.ordinal()][getNextTurn().direction.ordinal()];
    }

    class MoveBackward implements Delegate {

        int countDownMove1;
        int countDownMove2;

        MoveBackward() {
            tickTr = 0;
            turning = false;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            System.out.println("Begin move backward");
            move.setEnginePower(0);
            countDownMove1 = staleMoveDelay;
            countDownMove2 = staleMoveDelay;
        }

        @Override
        public void move() {
            if (countDownMove1 > 0) {
                countDownMove1--;
                tickTr = 0;//Prevent stale check
            }
            double offsAngle = getOffsetAngle() / ANGLE_90;

            if (deltaBack() < tileSize * 0.2) {
                move.setWheelTurn(-offsAngle);
                if (countDownMove1 <= 0)
                    move.setEnginePower(-1.0D);
            } else {
                if (countDownMove2 > 0) {
                    countDownMove2--;
                    tickTr = 0;//Prevent stale check
                }
                move.setWheelTurn(offsAngle);
                if (countDownMove2 <= 0)
                    delegate = new MoveForward();
            }
            if (isStale())
                delegate = new MoveForward();
        }
    }

    void checkCanThrowProjectile() {
        if (self.getProjectileCount() > 0)
            for (Car car : world.getCars()) {
                if (self.getId() != car.getId()) {
                    if (abs(self.getAngleTo(car)) < abs(ANGLE_15)) {
                        if (abs(self.getDistanceTo(car)) < self.getHeight() * 6) {
                            move.setThrowProjectile(true);
                        }
                    }
                }
            }
    }

    void checkNitro() {
        if (!canNitro || turning || brakeOn || nitroOn || self.getNitroChargeCount() < 1) {
            return;
        }

        if (getNextTurn().distance > tileSize * 4) {
            System.out.println(getNextTurn().distance);
            System.out.println("Use nitro!!!");
            nitroOn = true;
            move.setUseNitro(true);
            nitroCountDownStart = curentTick;
        }
    }

    void checkBrake() {
        brakeOn = false;

        if (turning || getNextTurn().distance < tileSize)
            brakeOn = speed > turnMaxSpeed;
        if (brakeOn) {
            System.out.println("Bake ON! " + speed);
            move.setEnginePower(0.2);
            move.setBrake(true);
        }
    }

    boolean checkCloseAngle(double angle, double max) {
        for (Car car : world.getCars()) {
            if (self.getId() != car.getId()) {
                if (abs(abs(angle) - abs(self.getAngleTo(car))) < max) {
                    System.out.println("Close!! " + angle + " " + self.getAngleTo(car));
                    return true;
                }
            }
        }
        return false;
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
        boolean hasCarInBack;
        double[] center;

        Turn() {
            tickTr = 0;
            turnAngleBegin = getRightAngle();
            needAngle = turnAngleBegin + get() * ANGLE_90;
            System.out.println("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + needAngle);
            hasCarInBack = checkCloseAngle(ANGLE_180, ANGLE_15);
            center = getTurnCenterCoord();
            System.out
                    .println("New turn " + (tileSize * cx + tileSize / 2) + "," + (tileSize * cy + tileSize / 2) + " center: " + center[0] + ", " + center[1]);
            turnBegin(this);

        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkBrake();

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));

            if (delta >= ANGLE_45) {
                turnHalf(this);
            }

            if (delta >= ANGLE_80) {
                System.out.println("Angle is " + self.getAngle() + ", delta is " + delta + " stop turn");
                turnEnd(this);
            } else {
                System.out.println("D: " + self.getDistanceTo(center[0], center[1]) + ", R:" + tileSize);
                if (self.getDistanceTo(center[0], center[1]) > tileSize * turnRadiusFactor) {
                    move.setWheelTurn(get());
                    checkBrake();
                } else {
                    move.setWheelTurn(0.5 * get());
                    move.setBrake(false);
                    move.setEnginePower(1.0);
                }
            }

            if (isStale()) {
                System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
                print();
                delegate = new MoveBackward();
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

    TileType getNextTile(int d) {
        return getNextTileOf(cx, cy, d);
    }

    TileType getNextTileOf(int x, int y, int d) {
        if (isUp() && cy >= d)
            y -= d;
        else if (isDown() && cy < world.getHeight() - d)
            y += d;
        else if (isLeft() && cx >= d)
            x -= d;
        else if (isRight() && cx < world.getWidth() - d)
            x += d;
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

    Direction resolveDirectionAfterNextTurn(TileType nextTile) {

        if (null != nextTile) {
            System.out.println("Turn: " + lastTile + " -> " + nextTile);
            if (nextTile == LEFT_TOP_CORNER) {
                if (isUp()) {
                    return RIGHT;
                }
                if (isLeft()) {
                    return DOWN;
                }
            }

            if (nextTile == RIGHT_TOP_CORNER) {
                if (isRight()) {
                    return DOWN;
                }
                if (isUp()) {
                    return LEFT;
                }
            }

            if (nextTile == RIGHT_BOTTOM_CORNER) {
                if (isDown()) {
                    return LEFT;
                }
                if (isRight()) {
                    return UP;
                }
            }

            if (nextTile == LEFT_BOTTOM_CORNER) {
                if (isLeft()) {
                    return UP;
                }
                if (isDown()) {
                    return RIGHT;
                }
            }

            if (nextTile == TileType.BOTTOM_HEADED_T) {
                if (isUp()) {
                    return self.getNextWaypointX() > cx ? RIGHT : LEFT;
                }
            }
            if (nextTile == TileType.TOP_HEADED_T) {
                if (isDown()) {
                    return self.getNextWaypointX() > cx ? RIGHT : LEFT;
                }
            }

        }
        return null;
    }

    NextTurn getNextTurn() {
        if (null == nextTurn) {
            flow(nextTurnFinder);
        }
        return null == nextTurn ? currentTurn : nextTurn;
    }

    private final Flow nextTurnFinder = new Flow() {

        @Override
        public boolean apply(TileType tileType, int x, int y, int c) {
            if (c > 0 && isTurnTile(tileType)) {
                nextTurn = new NextTurn();
                nextTurn.x = tileSize * x + tileSize / 2;
                nextTurn.y = tileSize * y + tileSize / 2;
                nextTurn.tileX = x;
                nextTurn.tileY = y;
                nextTurn.tile = tileType;
                nextTurn.direction = resolveDirectionAfterNextTurn(tileType);
                nextTurn.distance = self.getDistanceTo(nextTurn.x, nextTurn.y);
                System.out.println("Find next turn : " + nextTurn.tile);
                return false;
            }
            return true;
        }

    };

    Bonus findNextBonus() {
        double closestXY = Double.MAX_VALUE;
        Bonus res = null;
        for (Bonus b : world.getBonuses()) {
            double dist = self.getDistanceTo(b);
            if (abs(self.getAngleTo(b)) < ANGLE_45
                    && dist < getNextTurn().distance) {
                if (dist < closestXY) {
                    res = b;
                }
            }
        }
        return res;
    }

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
        if (null != nextTurn)
            currentTurn = nextTurn;
        turning = true;
        turningHalf = false;
    }

    void turnHalf(Turn turn) {
        if (!turningHalf) {
            turningHalf = true;
            nextTurn = null;
            if (isUp())
                direction = turn.isLeft() ? LEFT : RIGHT;
            else if (isDown())
                direction = turn.isLeft() ? RIGHT : LEFT;
            else if (isLeft())
                direction = turn.isLeft() ? DOWN : UP;
            else if (isRight())
                direction = turn.isLeft() ? UP : DOWN;

            turnCount++;

            if (turnCount > 1 && turn.hasCarInBack)
                move.setSpillOil(true);
        }
    }

    void turnEnd(Turn turn) {
        System.out.println("Turn end " + direction);
        //flow(nextTurnFinder);
        delegate = null;
        checkTurn();
        turning = false;
    }

    double getEndSpeed(double dist) {
        if (0 == accel)
            return speed;
        return sqrt(speed * speed + 2 * dist * accel);
    }

    double[] getTurnCenterCoord() {
        System.out.println(TURN_CENTER_OFFSET_X[direction.ordinal()][getNextTurn().direction.ordinal()]);
        double x = tileSize * cx + tileSize / 2 + TURN_CENTER_OFFSET_X[direction.ordinal()][getNextTurn().direction.ordinal()] * tileSize;
        double y = tileSize * cy + tileSize / 2 + TURN_CENTER_OFFSET_Y[direction.ordinal()][getNextTurn().direction.ordinal()] * tileSize;
        return new double[] {x, y};
    }

}
