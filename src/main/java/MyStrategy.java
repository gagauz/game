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
import model.Bonus;
import model.Car;
import model.Direction;
import model.Game;
import model.Move;
import model.TileType;
import model.World;

public final class MyStrategy implements Strategy {

    static double insideTurnFactor = 1.0;//0..1
    static double outsideTurnFactor = 1;//0..1
    static double straightFactor = 0.002;

    static double turnMaxSpeed = 15;
    static double driftSpeed = 28;
    static boolean canNitro = true;
    static int staleTickCount = 50;
    static double staleSpeedValue = 0.1;
    static double staleMoveValue = 1;
    static double turnRadiusFactor = 3;
    static double maxSpd = 1000;
    static double margins = 80;

    private static final double[] OFFSET = new double[] {1, -1, -1, 1};

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
    private static final double ANGLE_60 = PI / 3;
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
            margins = game.getTrackTileMargin();
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
        if (speed > maxSpd) {
            move.setEnginePower(0.1);
        }
        //        checkMargins();
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

        int tileAhead = speed > driftSpeed ? 2 : 1;

        // TODO Если ускорение - искать раньше 
        if (turning)
            nextTile = getNextTileOf(currentTurn.tileX, currentTurn.tileY, 1);
        else
            nextTile = getNextTile(tileAhead);

        if (nextTile != lastTile) {
            double offs = getCenterOffset() * TURN_ANGLE_OUT[direction.ordinal()][getNextTurn().direction.ordinal()];
            double dist = getNextTurn().distance - tileSize / 2;
            double start = tileSize / 2;
            //Чем ближе к краю и чем меньше скорость, тем больше задержка

            double offsAngle = getOffsetAngle();

            //Фактор скорости
            double spdFact = speed / turnMaxSpeed * tileSize * (speed - 0.7 * turnMaxSpeed) / turnMaxSpeed * 3;
            if (spdFact > 0)
                start += spdFact;
            //Фактор смещения
            start -= offs * 2.0;
            //Фактор угла вхождения в поворот
            start -= -offsAngle * OFFS_ANGLE_SIGN[direction.ordinal()][getNextTurn().direction.ordinal()] * 2.0;

            if (cx == currentTurn.tileX && cy == currentTurn.tileY) {
                start = 1;
                dist = 0;
            }

            if (dist < start) {

                Direction dir = resolveDirectionAfterNextTurn(nextTile);
                if (null != dir && dir != direction) {
                    System.out.println("Turn detected: dir=" + direction + ", next dir=" + dir + ", lastTile=" + lastTile + " nextTile= " + nextTile
                            + ", start=" + start + ", dist=" + dist + ", speed=" + speed + ", tileAhead=" + tileAhead + ", turning=" + turning);
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
            double delta1 = 3 * getOffsetAngle();

            double offset = getCenterOffset();
            //            System.out.println("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

            double delta2 = straightFactor * OFFSET[direction.ordinal()] * offset;
            //            System.out.println("Dir o1=" + delta1 + ", o2=" + delta2);

            delta1 += delta2;
            //            move.setEnginePower(0.2 + abs(ANGLE_90 - abs(offset)) / ANGLE_90);
            move.setWheelTurn(delta1);
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
        if (need == ANGLE_180) {
            need = 0;
            real = real + (real > 0 ? -ANGLE_180 : ANGLE_180);
        }

        double offsAngle = (need - real);
        return offsAngle;
    }

    class MoveBackward implements Delegate {

        int countDownMove1;
        int countDownMove2;

        MoveBackward() {
            tickTr = 0;
            turning = false;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            System.out.println("Begin move backward dir: " + direction + " angle:" + self.getAngle());
            move.setEnginePower(0);
            countDownMove1 = staleMoveDelay;
            countDownMove2 = 2 * staleMoveDelay;
        }

        @Override
        public void move() {
            if (countDownMove1 > 0) {
                countDownMove1--;
                tickTr = 0;//Prevent stale check
            }
            double offsAngle = getOffsetAngle() / ANGLE_90;

            if (deltaBack() < tileSize * 0.4) {
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

    void checkMargins() {
        if (turning)
            return;
        double correction = 0.2;
        double dR = self.getX() + self.getWidth() / 2;
        double dL = self.getX() - self.getWidth() / 2;
        double dU = self.getY() - self.getWidth() / 2;
        double dD = self.getY() + self.getWidth() / 2;
        double mR = (cx + 1) * tileSize - game.getTrackTileMargin();
        double mL = cx * tileSize + game.getTrackTileMargin();
        double mU = cy * self.getY() + game.getTrackTileMargin();
        double mD = (cy + 1) * self.getY() - game.getTrackTileMargin();
        if (isUp()) {
            if (dR > mR)
                move.setWheelTurn(-correction);
            else if (dL < mL)
                move.setWheelTurn(correction);
        } else if (isDown()) {
            if (dR > mR)
                move.setWheelTurn(correction);
            else if (dL < mL)
                move.setWheelTurn(-correction);
        } else if (isRight()) {
            if (dU < mU)
                move.setWheelTurn(correction);
            else if (dD > mD)
                move.setWheelTurn(-correction);
        } else if (isLeft()) {
            if (dU < mU)
                move.setWheelTurn(-correction);
            else if (dD > mD)
                move.setWheelTurn(correction);
        }
        System.out.println("dR=" + dR + ", mR=" + mR + ", dL=" + dL + ", mL=" + mL + ", dU=" + dU + ", mU=" + mU + ", dD=" + dD + ", mD=" + mD);
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
            hasCarInBack = checkCloseAngle(ANGLE_180, ANGLE_45);
            center = getTurnTileInsideCornerCoord();
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

            System.out.println(tileSize);

            if (turnCount > 1 && hasCarInBack
                    && cx == currentTurn.tileX
                    && cy == currentTurn.tileY
                    && self.getDistanceTo(center[0], center[1]) < self.getWidth() * 2)
                move.setSpillOil(true);

            if (delta >= ANGLE_45) {
                turnHalf(this);
            }

            if (delta >= ANGLE_80) {
                System.out.println("Angle is " + self.getAngle() + ", delta is " + delta + " stop turn");
                turnEnd(this);
            } else {
                double angle = abs(self.getAngleTo(center[0], center[1]));
                System.out.println("D: " + angle + ", Center:" + center[0] + ", " + center[1] + ", self: "
                        + self.getX() + ", " + self.getY());

                if (angle > ANGLE_15) {
                    move.setWheelTurn(get());
                    checkBrake();
                } else {
                    if (speed < driftSpeed)
                        move.setWheelTurn(0.1);
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
            if (abs(self.getAngleTo(b)) < ANGLE_60
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
            System.out.print("turnHalf: dir=" + direction);
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

            System.out.println(", next dir=" + direction);

            if (isTurnTile(getNextTileOf(currentTurn.tileX, currentTurn.tileY, 1))) {
                System.out.println("Next tile is turn! Stop turn.");
                delegate = new MoveForward();
            }
        }
    }

    void turnEnd(Turn turn) {
        System.out.println("turnEnd: dir=" + direction);
        //flow(nextTurnFinder);
        delegate = null;
        turning = false;
    }

    double getEndSpeed(double dist) {
        if (0 == accel)
            return speed;
        return sqrt(speed * speed + 2 * dist * accel);
    }

    double[] getTurnTileInsideCornerCoord() {
        double x = tileSize * getNextTurn().tileX + tileSize / 2 + TURN_CENTER_OFFSET_X[direction.ordinal()][getNextTurn().direction.ordinal()]
                * (tileSize / 2 - margins);
        double y = tileSize * getNextTurn().tileY + tileSize / 2 + TURN_CENTER_OFFSET_Y[direction.ordinal()][getNextTurn().direction.ordinal()]
                * (tileSize / 2 - margins);
        return new double[] {x, y};
    }

    public static void main(String[] args) {

        int cx = 0;
        int cy = 2;

        Direction d1 = LEFT;
        Direction d2 = UP;

        tileSize = 800;

        double x = tileSize * cx + tileSize / 2 + TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]
                * (tileSize / 2 - 80);
        double y = tileSize * cy + tileSize / 2 + TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]
                * (tileSize / 2 - 80);

        System.out.println(TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]);
        System.out.println(TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]);

        System.out.println(x);
        System.out.println(y);
    }

}
