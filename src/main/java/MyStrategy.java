import model.*;

import static java.lang.Math.*;
import static model.Direction.*;
import static model.TileType.*;

public final class MyStrategy implements Strategy {

    static double insideTurnFactor = 0.85;//0..1
    static double outsideTurnFactor = 0.8;//0..1
    static double mobility_factor = 0.0020;

    static double turn_max_speed = 15;
    static double driftSpeed = 28;
    static boolean canNitro = false;
    static int staleTickCount = 50;
    static double staleSpeedValue = 0.1;
    static double staleMoveValue = 2;
    static double turnRadiusFactor = 3;
    static double max_speed = 50;
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
    private static final double ANGLE_120 = 2 * PI / 3;
    private static final double ANGLE_90 = PI / 2;
    private static final double ANGLE_20 = ANGLE_90 / 90 * 20;
    private static final double ANGLE_60 = PI / 3;
    private static final double ANGLE_45 = PI / 4;
    private static final double ANGLE_30 = PI / 6;
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
    private double mobility = mobility_factor;
    private double maxSpeed = max_speed;
    private double turnMaxSpeed = turn_max_speed;

    class TileXY {
        final int x;
        final int y;

        TileXY(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    class NextTurn {
        final double x;
        final double y;
        final int tileX;
        final int tileY;
        final TileType tile;
        final Direction direction;
        double dist;

        NextTurn(int x, int y, Direction direction) {
            this.tileX = x;
            this.tileY = y;
            this.tile = world.getTilesXY()[x][y];
            this.x = getTileCenter(tileX);
            this.y = getTileCenter(tileY);
            this.direction = resolveDirectionAfterTurn(this.tile, direction);
        }

        double distanceTo() {
            return dist;
        }
    }

    private NextTurn currentTurn;
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
    private TileType lastTurnTile = EMPTY;
    private TileType nextTile = EMPTY;

    //    

    @Override
    public void move(Car self, World world, Game game, Move move) {

        if (0 == tileSize) {
            tileSize = (int) game.getTrackTileSize();
        }
        cx = getTileXY(self.getX());
        cy = getTileXY(self.getY());

        this.curentTick++;
        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;

        if (null == direction) {
            direction = world.getStartingDirection();
            staleMoveDelay = (int) (1.0 / game.getCarWheelTurnChangePerTick());
            margins = game.getTrackTileMargin();
            currentTurn = new NextTurn(cx, cy, direction);
        }

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

        defineCenter();

        if (null != currentTurn) {
            currentTurn.dist = self.getDistanceTo(currentTurn.x, currentTurn.y);
        }
        if (null != nextTurn) {
            nextTurn.dist = self.getDistanceTo(nextTurn.x, nextTurn.y);
        }

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
        if (speed > 1.2 * maxSpeed) {
            move.setEnginePower(maxSpeed / speed);
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

    void checkStale() {

        if ((deltaMove() < staleMoveValue)
                && (speed < staleSpeedValue)
                && (accel < staleSpeedValue)
                && tickTr > staleTickCount) {

            System.out.println("Detect stale : Dm=" + deltaMove() + ", Ds=" + deltaSpeed());
            staleOn = true;
            brakeOn = false;
            move.setBrake(false);
            if (delegate instanceof MoveBackward) {
                tickTr = 0;
                delegate = ((MoveBackward) delegate).lastDelegate;
            } else {

                delegate = new MoveBackward(delegate);
            }
        }
    }

    void checkTurn() {

        if (staleOn)
            return;

        int tileAhead = speed > driftSpeed ? 2 : 1;

        int[] nextTileXY;

        if (turning)
            nextTileXY = getNextTileXY(currentTurn.tileX, currentTurn.tileY, 1, direction);
        else
            nextTileXY = getNextTileXY(cx, cy, tileAhead, direction);

        nextTile = world.getTilesXY()[nextTileXY[0]][nextTileXY[1]];

        if (EMPTY == nextTile) {
            nextTileXY = new int[] {cx, cy};
            nextTile = world.getTilesXY()[nextTileXY[0]][nextTileXY[1]];

        }

        Delegate old = delegate;

        if (nextTile != lastTurnTile) {
            if (isTurnTile(nextTile, direction)) {
                nextTurn = new NextTurn(nextTileXY[0], nextTileXY[1], direction);
                double offs = getCenterOffset() * TURN_ANGLE_OUT[direction.ordinal()][nextTurn.direction.ordinal()];
                double dist = getNextTurn().distanceTo();
                double start = tileSize / 2;
                //Чем ближе к краю и чем меньше скорость, тем больше задержка

                double offsAngle = getOffsetAngle();

                //Фактор скорости
                double spdFact = speed / turnMaxSpeed * tileSize * (speed - 0.7 * turnMaxSpeed) / turnMaxSpeed * 3;
                if (spdFact > 0)
                    start += spdFact;
                //Фактор смещения
                start -= offs * 3.0;
                //Фактор угла вхождения в поворот
                start -= -offsAngle * OFFS_ANGLE_SIGN[direction.ordinal()][getNextTurn().direction.ordinal()] * 2.0;

                // Если уже внутри поворота - никаких задержек
                if (cx == getNextTurn().tileX && cy == getNextTurn().tileY) {
                    start = 1;
                    dist = 0;
                }

                if (dist < start) {

                    Direction dir = resolveDirectionAfterTurn(nextTile, direction);
                    if (null != dir && dir != direction) {
                        System.out.println("Turn detected: dir=" + direction + ", next dir=" + dir + ", lastTile=" + lastTurnTile + " nextTile= " + nextTile
                                + ", start=" + start + ", dist=" + dist + ", speed=" + speed + ", tileAhead=" + tileAhead + ", turning=" + turning);
                        if (isUp()) {
                            if (RIGHT == dir)
                                delegate = new TurnRight(nextTurn);
                            if (LEFT == dir)
                                delegate = new TurnLeft(nextTurn);
                        } else if (isDown()) {
                            if (RIGHT == dir)
                                delegate = new TurnLeft(nextTurn);
                            if (LEFT == dir)
                                delegate = new TurnRight(nextTurn);
                        } else if (isLeft()) {
                            if (UP == dir)
                                delegate = new TurnRight(nextTurn);
                            if (DOWN == dir)
                                delegate = new TurnLeft(nextTurn);
                        } else if (isRight()) {
                            if (DOWN == dir)
                                delegate = new TurnRight(nextTurn);
                            if (UP == dir)
                                delegate = new TurnLeft(nextTurn);
                        }
                    }
                }
                if (delegate != old) {
                    lastTurnTile = nextTile;
                }
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
        //        if (!turning) {
        double delta1 = getOffsetAngle();

        double offset = getCenterOffset();
        //            System.out.println("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

        double delta2 = mobility * OFFSET[direction.ordinal()] * offset;
        //            System.out.println("Dir o1=" + delta1 + ", o2=" + delta2);

        delta1 += delta2;
        //            move.setEnginePower(0.2 + abs(ANGLE_90 - abs(offset)) / ANGLE_90);
        move.setWheelTurn(delta1);
        //        }
    }

    void avoidCollision(double x, double y, double angle, double dist) {
        double a = self.getAngleTo(x, y);
        double d = self.getDistanceTo(x, y);
        if (abs(a) < angle && d < dist) {
            move.setWheelTurn(2 * -a * (angle - abs(a)) / angle);
        }
    }

    void avoidCollisionWithCars() {
        double avoidDist = tileSize;
        double avoidAngle = ANGLE_15;
        Car carToAvoid = null;
        for (Car car : world.getCars()) {
            if (self.getId() != car.getId()) {
                double d = self.getDistanceTo(car);
                double a = self.getAngleTo(car);
                if (d < avoidDist && abs(a) < avoidAngle) {
                    carToAvoid = car;
                    avoidDist = d;
                }
            }
        }
        if (null != carToAvoid) {
            double a = self.getAngleTo(carToAvoid);
            move.setWheelTurn(2 * -a * (avoidAngle - abs(a)) / avoidAngle);
        }
    }

    void defineCenter() {
        if (!turning) {

            centerX = cx * tileSize + tileSize / 2;
            centerY = cy * tileSize + tileSize / 2;
            maxSpeed = max_speed;
            mobility = mobility_factor;
            turnMaxSpeed = turn_max_speed;
            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            boolean increaseMobility = false;

            Bonus b = null;

            if (null != getNextTurn()) {
                if (nextTurn.distanceTo() < tileSize * 4) {
                    NextTurn nt = getTurnAfterTurn(nextTurn);
                    double m = getNextTurn().distanceTo() < tileSize * 1.2 * speed / turnMaxSpeed ? -insideTurnFactor : outsideTurnFactor;

                    if (null != nt) {
                        if (nt.direction != direction) {
                            m = outsideTurnFactor;
                        } else {
                            m = -outsideTurnFactor;
                        }
                        increaseMobility = true;
                        turnMaxSpeed = turn_max_speed * 0.6;
                    }

                    centerX += m * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().direction.ordinal()] * self.getWidth();
                    centerY += m * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().direction.ordinal()] * self.getWidth();
                }
            }
            if (!increaseMobility) {
                if ((b = findNextBonus()) != null) {
                    double dx = b.getX() - centerX;
                    double dy = b.getY() - centerY;
                    dx = between(abs(dx), self.getWidth() / 3, tileSize / 2) ? dx : 0;
                    dy = between(abs(dy), self.getWidth() / 3, tileSize / 2) ? dy : 0;
                    centerX += dx;
                    centerY += dy;
                    if (dx != 0 || dy != 0) {
                        increaseMobility = true;
                    }
                    System.out.println("Bonus!! + " + b.getType() + " dx=" + dx + ", dy=" + dy);
                }
            }
            if (increaseMobility) {
                mobility = mobility_factor * 2;
                maxSpeed = turnMaxSpeed;
            }
        }
    }

    NextTurn getTurnAfterTurn(NextTurn turn) {
        if (null != turn) {
            int ntX = turn.tileX;
            int ntY = turn.tileY;
            Direction ntD = turn.direction;
            int[] xy = getNextTileXY(ntX, ntY, 1, ntD);
            TileType tileAfterTurn = world.getTilesXY()[xy[0]][xy[1]];
            if (isTurnTile(tileAfterTurn, ntD)) {
                return new NextTurn(xy[0], xy[1], ntD);
            }
        }
        return null;
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
            avoidCollisionWithCars();
            checkStale();
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
        if (offsAngle < -ANGLE_180) {
            offsAngle += ANGLE_180;
        }
        if (offsAngle > ANGLE_180) {
            offsAngle -= ANGLE_360;
        }
        return offsAngle;
    }

    class MoveBackward implements Delegate {

        final Delegate lastDelegate;
        int countDownMove1;
        int countDownMove2;

        MoveBackward(Delegate lastDelegate) {
            this.lastDelegate = lastDelegate;
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
                move.setBrake(true);
            }
            double offsAngle = getOffsetAngle() / ANGLE_90;

            checkStraight();

            if (deltaBack() < tileSize / 2) {
                move.setWheelTurn(-offsAngle);
                if (countDownMove1 <= 0)
                    move.setEnginePower(-1.0D);
            } else {
                if (countDownMove2 > 0) {
                    move.setBrake(true);
                    countDownMove2--;
                    tickTr = 0;//Prevent stale check
                }
                move.setWheelTurn(offsAngle);
                if (countDownMove2 <= 0)
                    delegate = lastDelegate;
            }
            checkStale();
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

        if (getNextTurn().distanceTo() > tileSize * 4) {
            System.out.println(getNextTurn().distanceTo());
            System.out.println("Use nitro!!!");
            nitroOn = true;
            move.setUseNitro(true);
            nitroCountDownStart = curentTick;
        }
    }

    void checkBrake() {
        brakeOn = false;

        if (turning || null != nextTurn && nextTurn.distanceTo() < tileSize / 2 * (speed / turnMaxSpeed))
            brakeOn = speed > turnMaxSpeed;
        if (brakeOn) {
            System.out.println("Bake ON! " + speed);
            move.setEnginePower(0.5);
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

        TurnLeft(NextTurn nextTurn) {
            super(nextTurn);
        }

        @Override
        double get() {
            return -1.0;
        }
    }

    class TurnRight extends Turn {

        TurnRight(NextTurn nextTurn) {
            super(nextTurn);
        }

        @Override
        double get() {
            return 1.0;
        }
    }

    abstract class Turn implements Delegate {

        double needAngle;
        boolean hasCarInBack;
        boolean insideTurnTile;
        double[] center;
        double endAndle = ANGLE_60;

        Turn(NextTurn nextTurn) {
            tickTr = 0;
            turnAngleBegin = getRightAngle();
            needAngle = turnAngleBegin + get() * ANGLE_90;
            System.out.println("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + needAngle);
            hasCarInBack = checkCloseAngle(ANGLE_180, ANGLE_45);
            currentTurn = nextTurn;
            center = getTurnTileInsideCornerCoord(currentTurn);
            turning = true;
            turningHalf = false;
            insideTurnTile = cx == currentTurn.tileX && cy == currentTurn.tileY;
            System.out
                    .println("Start new turn " + (tileSize * cx + tileSize / 2) + "," + (tileSize * cy + tileSize / 2) + " center: " + center[0] + ", "
                            + center[1]);

            NextTurn tAt = getTurnAfterTurn(nextTurn);
            if (null != tAt) {
                endAndle = tAt.direction == direction ? ANGLE_60 : ANGLE_120;
            }
        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkBrake();

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));

            boolean inside = cx == currentTurn.tileX && cy == currentTurn.tileY;

            if (!insideTurnTile && inside) {
                insideTurnTile = true;
            }

            // Установить новое направление
            if (insideTurnTile) {
                direction = currentTurn.direction;
            }

            if (turnCount > 1 && hasCarInBack
                    && cx == currentTurn.tileX
                    && cy == currentTurn.tileY
                    && self.getDistanceTo(center[0], center[1]) < self.getWidth() * 2)
                move.setSpillOil(true);

            if (delta >= ANGLE_45) {
                turnHalf(this);
            }

            if (delta >= endAndle ||
                    (insideTurnTile && !inside)) {
                System.out.println("Turn finish: Angle is " + self.getAngle() + ", delta is " + delta);
                turnEnd(this);
            } else {
                move.setWheelTurn(get());
                checkBrake();
            }

            avoidCollisionWithCars();
            avoidCollision(center[0], center[1], ANGLE_30, tileSize);
            checkStale();
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

    int[] getNextTileXY(int fromX, int fromY, int delta, Direction dir) {
        if (UP == dir && fromY >= delta)
            fromY -= delta;
        else if (DOWN == dir && fromY < world.getHeight() - delta)
            fromY += delta;
        else if (LEFT == dir && fromX >= delta)
            fromX -= delta;
        else if (RIGHT == dir && fromX < world.getWidth() - delta)
            fromX += delta;
        return new int[] {fromX, fromY};
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

    Direction resolveDirectionAfterTurn(TileType nextTile, Direction dir) {

        if (null != nextTile) {
            if (nextTile == LEFT_TOP_CORNER) {
                if (UP == dir) {
                    return RIGHT;
                }
                if (LEFT == dir) {
                    return DOWN;
                }
            }

            if (nextTile == RIGHT_TOP_CORNER) {
                if (RIGHT == dir) {
                    return DOWN;
                }
                if (UP == dir) {
                    return LEFT;
                }
            }

            if (nextTile == RIGHT_BOTTOM_CORNER) {
                if (DOWN == dir) {
                    return LEFT;
                }
                if (RIGHT == dir) {
                    return UP;
                }
            }

            if (nextTile == LEFT_BOTTOM_CORNER) {
                if (LEFT == dir) {
                    return UP;
                }
                if (DOWN == dir) {
                    return RIGHT;
                }
            }

            if (nextTile == BOTTOM_HEADED_T) {
                if (UP == dir) {
                    return self.getNextWaypointX() > cx ? RIGHT : LEFT;
                }
            }
            if (nextTile == TOP_HEADED_T) {
                if (DOWN == dir) {
                    return self.getNextWaypointX() > cx ? RIGHT : LEFT;
                }
            }
            if (nextTile == LEFT_HEADED_T) {
                if (RIGHT == dir) {
                    return self.getNextWaypointY() > cy ? DOWN : UP;
                }
            }
            if (nextTile == RIGHT_HEADED_T) {
                if (LEFT == dir) {
                    return self.getNextWaypointY() > cy ? DOWN : UP;
                }
            }

        }
        return direction;
    }

    NextTurn getNextTurn() {
        if (null == nextTurn) {
            nextTurn = findNextTurnFrom(cx, cy, direction, Integer.MAX_VALUE);
        }
        return nextTurn;
    }

    NextTurn findNextTurnFrom(int fromX, int fromY, Direction dir, int limit) {
        int d = DOWN == dir || RIGHT == dir ? 1 : -1;

        int dX = UP == dir || DOWN == dir ? 0 : d;
        int dY = UP == dir || DOWN == dir ? d : 0;
        int maxX = world.getWidth();
        int maxY = world.getHeight();
        for (int x = fromX, y = fromY, c = 0; x < maxX && y < maxY && x > -1 && y > -1 && c < limit; x = x + dX, y = y + dY, c++) {
            TileType tile = world.getTilesXY()[x][y];
            if (c > 0 && isTurnTile(tile, dir)) {
                return new NextTurn(x, y, dir);
            }
        }
        return null;
    }

    double deltaDist(Unit unit) {
        double unitXY = isHorizontal() ? unit.getY() : unit.getX();
        double selfXY = isHorizontal() ? self.getY() : self.getX();
        return unitXY - selfXY;
    }

    Bonus findNextBonus() {
        double closestXY = Double.MAX_VALUE;
        Bonus res = null;
        for (Bonus b : world.getBonuses()) {
            double dist = self.getDistanceTo(b);
            if (abs(self.getAngleTo(b)) < ANGLE_90
                    && dist < tileSize
                    && abs(deltaDist(b)) < (tileSize - 2 * margins - self.getWidth() / 2)) {
                if (dist < closestXY) {
                    closestXY = dist;
                    res = b;
                }
            }
        }
        return res;
    }

    boolean isTurnTile(TileType type, Direction dir) {
        boolean turn = (LEFT_TOP_CORNER == type)
                || (LEFT_BOTTOM_CORNER == type)
                || (RIGHT_BOTTOM_CORNER == type)
                || (RIGHT_TOP_CORNER == type);
        if (!turn) {
            turn = (LEFT_HEADED_T == type && RIGHT == dir)
                    || (RIGHT_HEADED_T == type && LEFT == dir)
                    || (TOP_HEADED_T == type && DOWN == dir)
                    || (BOTTOM_HEADED_T == type && UP == dir);
        }
        return turn;
    }

    void turnHalf(Turn turn) {
        if (!turningHalf) {
            System.out.print("turnHalf: dir=" + direction);
            turningHalf = true;
            turnCount++;
        }
    }

    void turnEnd(Turn turn) {
        System.out.println("turnEnd: dir=" + direction);
        //flow(nextTurnFinder);
        currentTurn = null;
        delegate = null;
        turning = false;
        nextTurn = null;
    }

    double getEndSpeed(double dist) {
        if (0 == accel)
            return speed;
        return sqrt(speed * speed + 2 * dist * accel);
    }

    double[] getTurnTileInsideCornerCoord(NextTurn nextTurn) {
        double x = tileSize * nextTurn.tileX + tileSize / 2 + TURN_CENTER_OFFSET_X[direction.ordinal()][nextTurn.direction.ordinal()]
                * (tileSize / 2 - margins);
        double y = tileSize * nextTurn.tileY + tileSize / 2 + TURN_CENTER_OFFSET_Y[direction.ordinal()][nextTurn.direction.ordinal()]
                * (tileSize / 2 - margins);
        return new double[] {x, y};
    }

    private static double getTileCenter(int tileXY) {
        return tileSize * tileXY + tileSize / 2;
    }

    private static int getTileXY(double tileXY) {
        return (int) tileXY / tileSize;
    }

    public static void main(String[] args) {

        int cx = 0;
        int cy = 2;

        Direction d1 = LEFT;
        Direction d2 = UP;

        tileSize = 800;

        double x = getTileCenter(cx) + TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]
                * (tileSize / 2 - 80);
        double y = tileSize * cy + tileSize / 2 + TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]
                * (tileSize / 2 - 80);

        System.out.println(TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]);
        System.out.println(TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]);

        System.out.println(x);
        System.out.println(y);
    }

}
