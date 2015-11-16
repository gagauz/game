import model.*;

import static java.lang.Math.*;
import static model.Direction.*;
import static model.TileType.*;

public final class MyStrategy implements Strategy {
    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_120 = 2 * PI / 3.0;
    private static final double ANGLE_90 = PI / 2.0;
    private static final double ANGLE_30 = PI / 6.0;
    private static final double ANGLE_15 = PI / 12.0;
    private static final double ANGLE_5 = PI / 36.0;
    private static final double ANGLE_1 = PI / 180.0;

    static double mass_factor = 1.00 / 1250.00;

    static double inside_turn_factor = 0.2;//0..1
    static double outside_turn_factor = 1;//0..1
    static double mobility_factor = 0.45;
    static double mobility_2x_factor = 1.0;
    static double stability_factor = 2.5;
    static double turn_max_speed = 15;
    static double turn_max_angular_speed = 0.043;
    static double turn_enter_offset_tiles = 2;
    static double turn_end_delta_angle = ANGLE_90;
    static double drift_speed = 26;
    static boolean enable_nitro = false;
    static boolean enable_turn_enter_offset = true;
    static boolean enable_bonus_lookup = false;
    static boolean enable_car_avoidance = false;
    static boolean enable_corner_avoidance = true;
    static boolean enable_turn_enter_delay = false;
    static int stale_tick_count = 50;
    static double stale_speed_value = 0.1;
    static double stale_move_value = 2;
    static double max_speed = 1000;
    static double margins = 80;
    static double fromCenterToBorder = 400;
    static double fromCenterToBorderWithWidth = 400;

    private static final double[] OFFSET = new double[] {1, -1, -1, 1};

    private static final double[][] TURN_DIRECTION = new double[][] {
            //      L   R  U  D
            /* L */{0, 0, 1, -1},
            /* R */{0, 0, -1, 1},
            /* U */{-1, 1, 0, 0},
            /* D */{1, -1, 0, 0}
    };

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

    private static double tile_size;

    private Car self;
    private Car ally;
    private World world;
    private Game game;
    private Move move;

    private double lastX;
    private double lastY;
    private double lastSpeedX;
    private double lastSpeedY;
    private double maxSpeed = max_speed;
    private double turnMaxSpeed = turn_max_speed;
    private double fireMaxDistance;
    private double oilMaxDistance;
    private boolean isForward;
    private boolean isBackward;
    private boolean backwardOn;
    private boolean beforeTurnTile;
    private int tickWait = 0;

    enum DoubleTurn {
        NONE,
        U_TYPE,
        Z_TYPE;
    }

    class NextTurn {
        final double x;
        final double y;
        final double cornerX;
        final double cornerY;
        final int tileX;
        final int tileY;
        final TileType tile;
        final Direction afterDirection;
        double dist;
        double turnDirection = 0;
        int deltaTile = 1;
        DoubleTurn doubleTurn;
        boolean entered = false;

        NextTurn(int tileX, int tileY, Direction currentDirection) {
            this.tileX = tileX;
            this.tileY = tileY;
            this.tile = world.getTilesXY()[tileX][tileY];
            this.x = getTileCenter(tileX);
            this.y = getTileCenter(tileY);
            this.afterDirection = resolveDirectionAfterTurn(this.tile, currentDirection);
            this.dist = null != self ? self.getDistanceTo(x, y) : 0;
            this.entered = this.x == cx && this.y == cy;
            this.turnDirection = TURN_DIRECTION[currentDirection.ordinal()][afterDirection.ordinal()];
            NextTurn nextTurn = findNextTurnFrom(tileX, tileX, afterDirection, 1);
            double[] c = getTurnTileInsideCornerCoord(this, 0);
            this.cornerX = c[0];
            this.cornerY = c[1];
        }

        double distanceTo() {
            return dist;
        }
    }

    private NextTurn currentTurn;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;
    private boolean brakeOn;

    private PIDController pidOffset;
    private PIDController pidAngle;

    private int cx;
    private int cy;

    private double centerX;
    private double centerY;
    private double lastCenterX;
    private double lastCenterY;
    private Direction direction;
    private Bonus bonus;

    private int tickTr = 0;
    private int staleMoveDelay;
    int tickCount;
    double speedAvg;

    private Delegate delegate;

    //    

    static double A = 0.5;
    static double B = 4;
    static double C = 400;
    static double D = 1;
    static double E = 1;

    public MyStrategy() {
        this(0.002, 0.5, 0.5);
    }

    public MyStrategy(double a, double b, double c) {
        A = a;
        B = b;
        C = c;
    }

    void initPIDs() {
        pidAngle = new PIDController(0.001, 1, 1);
        pidAngle.setContinuous();
        pidAngle.setInputRange(-ANGLE_180, ANGLE_180);
        pidAngle.setOutputRange(-1.0, 1.0);
        pidAngle.setTolerance(5);
        pidAngle.setSetpoint(0);
        pidAngle.enable();

        pidOffset = new PIDController(0.001, 1, 1);
        pidOffset.setContinuous();
        pidOffset.setInputRange(-fromCenterToBorder, fromCenterToBorder);
        pidOffset.setOutputRange(-1.0, 1.0);
        pidOffset.setTolerance(5);
        pidOffset.setSetpoint(0);
        pidOffset.enable();
    }

    @Override
    public void move(Car self, World world, Game game, Move move) {

        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;

        if (0 == tile_size) {
            tile_size = (int) game.getTrackTileSize();

            staleMoveDelay = (int) (1.0 / game.getCarWheelTurnChangePerTick());
            margins = game.getTrackTileMargin();
            fromCenterToBorder = tile_size / 2 - margins;
            fromCenterToBorderWithWidth = fromCenterToBorder - self.getWidth() / 2;
            fireMaxDistance = tile_size * 1.5;
            oilMaxDistance = tile_size * 8;
            drift_speed = drift_speed / (mass_factor * self.getMass());
            mobility_2x_factor = mobility_2x_factor / (mass_factor * self.getMass());

        }

        cx = getTileXY(self.getX());
        cy = getTileXY(self.getY());

        if (null == direction) {
            direction = world.getStartingDirection();
            lastCenterX = getTileCenter(cx);
            lastCenterY = getTileCenter(cy);
            initPIDs();
        }
        setPIDCenter();
        centerX = lastCenterX;
        centerY = lastCenterY;

        maxSpeed = max_speed;
        turnMaxSpeed = turn_max_speed;

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

        defineCenter();

        if (null != currentTurn) {
            currentTurn.dist = isHorizontal() ? abs(self.getX() - currentTurn.x) : abs(self.getY() - currentTurn.y);
        }

        if (world.getTick() > game.getInitialFreezeDurationTicks()) {

            if (tickWait > 0) {
                --tickWait;
                return;
            }

            tickTr++;
            if (null == delegate) {
                delegate = new MoveForward();
            }
            delegate.move();
        } else {

            if (tickWait == 0 && !isCanUseNitro()) {
                tickWait = 20;
            }
        }

        lastX = self.getX();
        lastY = self.getY();

        lastSpeedX = self.getSpeedX();
        lastSpeedY = self.getSpeedY();

        //        //System.out.println("X=" + self.getX() + ", Y=" + self.getY() + ", A=" + self.getAngle() + ", Vx=" + self.getSpeedX() + ", Vy=" + self.getSpeedY());
        if (speed > maxSpeed) {
            //            //System.out.println("limit max speed " + (maxSpeed / speed));
            //            move.setEnginePower(maxSpeed / speed);
        }
        //        checkMargins();
        if (!self.isFinishedTrack() && world.getTick() > game.getInitialFreezeDurationTicks()) {
            tickCount++;
            speedAvg += speed;
        }
    }

    void changeDirection(Direction newDirection) {
        direction = newDirection;
        lastCenterX = getTileCenter(cx);
        lastCenterY = getTileCenter(cy);
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

        if ((deltaMove() < stale_move_value)
                && (speed < stale_speed_value)
                && (accel < stale_speed_value)
                && tickTr > stale_tick_count) {

            //System.out.println("Detect stale : dXY=" + deltaMove() + ", dV=" + deltaSpeed() + " brake=" + move.isBrake() + ", power=" + move.getEnginePower());
            debugOffsets();
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
        if (null == currentTurn) {
            NextTurn turn = findNextTurnFrom(cx, cy, direction, 2);
            if (null != turn) {
                currentTurn = turn;

            }
        } else {
            double deltaAngle = 0.0;
            if (!currentTurn.entered && cx == currentTurn.tileX && cy == currentTurn.tileY) {
                currentTurn.entered = true;
                direction = currentTurn.afterDirection;
            }

            if (currentTurn.entered) {
                deltaAngle = getOffsetCenterAxisAngle();
            }

            if (currentTurn.entered && (cx != currentTurn.tileX || cy != currentTurn.tileY || abs(deltaAngle) < ANGLE_30)) {
                currentTurn = null;
            }
        }
    }

    void setPIDCenter() {
        pidAngle.setSetpoint(getRightAngle(direction));
        double center = isVertical() ? lastCenterX : lastCenterY;
        pidOffset.setInputRange(center - fromCenterToBorder, center + fromCenterToBorder);
        pidOffset.setSetpoint(center);
    }

    void checkStraight() {
        double angle = self.getAngle();
        if (null != currentTurn) {
            angle += currentTurn.turnDirection * atan(fromCenterToBorder / (currentTurn.distanceTo() - tile_size / 3));
        }
        double e1 = 0;
        if (null != currentTurn && !currentTurn.entered) {
            pidAngle.setInput(angle);
            e1 = pidAngle.performPID();
        }

        pidOffset.setInput(isVertical() ? self.getX() : self.getY());
        double e2 = pidOffset.performPID();

        System.out.print(direction);
        System.out.print(" offset in=" + pidOffset.m_minimumInput + ", " + pidOffset.m_maximumInput);
        System.out.print(" offset out=" + pidOffset.m_minimumOutput + " , " + pidOffset.m_maximumOutput);
        System.out.print(" offset center=" + pidOffset.m_setpoint);
        System.out.print(" offset input=" + (isVertical() ? self.getX() : self.getY()));
        System.out.print(" offset output=" + e2);

        System.out.print(" angle in=" + pidAngle.m_minimumInput + ", " + pidAngle.m_maximumInput);
        System.out.print(" angle out=" + pidAngle.m_minimumOutput + ", " + pidAngle.m_maximumOutput);
        System.out.print(" angle center=" + pidAngle.m_setpoint);
        System.out.print(" angle input=" + self.getAngle());
        System.out.println("angle output=" + e1);

        double error = e1 + e2;

        move.setWheelTurn(-error);
    }

    void avoidCollisions(double x, double y) {

        double a1 = 0;
        double d1 = 0;
        double angle = ANGLE_15;
        double dist = tile_size;

        if (null != currentTurn && enable_corner_avoidance) {
            if (x > 0 && y > 0) {
                a1 = self.getAngleTo(x, y);
                d1 = self.getDistanceTo(x, y);
                double a2 = -a1 * OFFS_ANGLE_SIGN[direction.ordinal()][currentTurn.afterDirection.ordinal()];
                if (d1 < dist) {
                    if (a2 < -ANGLE_5) {
                        //System.out.println("Avoid corner x=" + x + ", y=" + y + ", a=" + a1 + ", d=" + d1
                        //                                + " self.x=" + self.getX()
                        //                                + " self.y=" + self.getY()
                        //                                + " correction=" + (-a1 * (angle - abs(a1)) / angle));
                        a1 = signum(a1);
                    }
                }
            }
        }

        Car carToAvoid = null;
        double d2 = 1;
        if (enable_car_avoidance) {
            for (Car car : world.getCars()) {
                if (self.getId() != car.getId()) {
                    d2 = self.getDistanceTo(car);
                    double a = self.getAngleTo(car);
                    if (d2 < dist && abs(a) < angle) {
                        carToAvoid = car;
                        dist = d2;
                    }
                }
            }
        }
        if (null != carToAvoid) {
            double a2 = self.getAngleTo(carToAvoid);
            a1 = a1 * cubic(d1, d2);
            a2 = -a2 * (angle - abs(a2)) / angle * quadro(d2, d1);

            //System.out.println("Avoid car a1=" + a1 + ", a2=" + a2 + ", d1=" + d1 + ", d2=" + d2);
            if (signum(a2) == signum(a1)) {
                a1 = maxByAbs(a1, a2);
            } else {
                a1 = a1 + a2;
            }

        }
        if (a1 != 0) {
            move.setWheelTurn(a1);
        }
    }

    void defineCenter() {
        if (!backwardOn && !beforeTurnTile) {

            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            boolean increaseMobility = false;

            if (enable_turn_enter_offset) {
                //                if (nextTurn.distanceTo() < tile_size * 4) {
                //                    double m = outside_turn_factor;
                //
                //                    if (null != doubleTurn) {
                //                        if (doubleTurn.afterDirection == direction) {
                //                            m = 0;
                //                        }
                //                        increaseMobility = true;
                //                    }
                //
                //                    centerX = lastCenterX + m * fromCenterToBorderWithWidth * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().afterDirection.ordinal()];
                //                    centerY = lastCenterY + m * fromCenterToBorderWithWidth * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().afterDirection.ordinal()];
                //                }
            }

            pickBonus();

            //Check limits
            double deltaCX = lastCenterX - centerX;
            if (abs(deltaCX) > fromCenterToBorderWithWidth) {
                centerX = lastCenterX + signum(centerX - lastCenterX) * fromCenterToBorderWithWidth;
            }
            double deltaCY = lastCenterY - centerY;
            if (abs(deltaCY) > fromCenterToBorderWithWidth) {
                centerY = lastCenterY + signum(centerY - lastCenterY) * fromCenterToBorderWithWidth;
            }
        }
    }

    void pickBonus() {
        if (enable_bonus_lookup && null == currentTurn) {
            if (null == bonus) {
                double closestXY = Double.MAX_VALUE;
                for (Bonus b : world.getBonuses()) {
                    if ((isForward && b.getType() == BonusType.OIL_CANISTER)
                            || (isForward && b.getType() == BonusType.REPAIR_KIT && ally.getDurability() < self.getDurability() && self.getDurability() > 0.5)) {
                        continue;
                    }
                    double dist = self.getDistanceTo(b);
                    if (abs(self.getAngleTo(b)) < ANGLE_90) {
                        if (dist < closestXY) {
                            double dx = lastCenterX - b.getX();
                            double dy = lastCenterY - b.getY();
                            boolean pick = (isVertical() && abs(dx) < fromCenterToBorder)
                                    || (isHorizontal() && abs(dy) < fromCenterToBorder);
                            if (pick && !isTurnTile(getTile(b.getX(), b.getY()), direction)) {
                                closestXY = dist;
                                bonus = b;
                            }
                        }
                    }
                }

            } else {
                //System.out.println("---------BONUS!");
                debugOffsets();

                if ((isUp() && self.getY() < bonus.getY())
                        || (isDown() && self.getY() > bonus.getY())
                        || (isLeft() && self.getX() < bonus.getX())
                        || (isRight() && self.getX() > bonus.getX())) {
                    bonus = null;
                } else {
                    centerX = bonus.getX()/* - bonus.getWidth() / 2 * signum(bonus.getX() - lastCenterX)*/;
                    centerY = bonus.getY()/* - bonus.getWidth() / 2 * signum(bonus.getY() - lastCenterY)*/;
                }
            }
        }
    }

    void print() {
        int x = self.getNextWaypointX();
        int y = self.getNextWaypointY();
        //System.out.println(world.getTilesXY()[x][y]);
    }

    class MoveForward implements Delegate {
        MoveForward() {
            print();
            tickTr = 0;
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkTurn();
            checkStraight();
            avoidCollisions(-1, -1);
            checkNitro();
            checkDistance();
            checkCanThrowProjectile();
            checkStale();
        }
    }

    double deltaBack() {
        return sqrt(pow(moveBackBeginX - self.getX(), 2) + pow(moveBackBeginY - self.getY(), 2));
    }

    double getOffsetCenterAxisAngle() {
        double need = getRightAngle(direction);
        double real = self.getAngle();
        //        if (need == ANGLE_180) {
        //            need = 0;
        //            real = real + (real > 0 ? -ANGLE_180 : ANGLE_180);
        //        }
        //
        //        double offsAngle = (need - real);
        //        if (offsAngle < -ANGLE_180) {
        //            offsAngle += ANGLE_180;
        //        }
        //        if (offsAngle > ANGLE_180) {
        //            offsAngle -= ANGLE_360;
        //        }
        return need - real;// offsAngle;
    }

    double getOffsetAngle() {
        double before = tile_size * 1.5;
        if (null != currentTurn) {
            if (!currentTurn.entered && currentTurn.distanceTo() > before) {
                double realAngle = self.getAngleTo(currentTurn.cornerX, currentTurn.cornerY);
                double needAngle = currentTurn.turnDirection * atan(fromCenterToBorder / currentTurn.distanceTo());
                return realAngle - needAngle;
            }
            if (currentTurn.entered || currentTurn.distanceTo() < before) {
                double angle = currentTurn.turnDirection * self.getAngleTo(currentTurn.cornerX, currentTurn.cornerY);
                return angle;
            }
        }
        return getOffsetCenterAxisAngle();
    }

    double getOffsetCenter() {
        if (null == currentTurn || !currentTurn.entered) {
            double offset = isHorizontal()
                    ? self.getY() - centerY
                    : self.getX() - centerX;
            return offset;
        }
        return 0;
    }

    double getOffsetCenterAxis() {

        double offset = isHorizontal()
                ? self.getY() - lastCenterY
                : self.getX() - lastCenterX;
        return offset;
    }

    class MoveBackward implements Delegate {

        final Delegate lastDelegate;
        int countDownMove1;
        int countDownMove2;
        double offsAngle = 0;

        MoveBackward(Delegate lastDelegate) {
            this.lastDelegate = lastDelegate;
            tickTr = 0;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            countDownMove1 = 2 * staleMoveDelay;
            countDownMove2 = 2 * staleMoveDelay;
            offsAngle = getOffsetAngle() > 0 ? 1 : -1;
            backwardOn = true;
        }

        @Override
        public void move() {
            if (countDownMove1 > 0) {
                countDownMove1--;
                tickTr = 0;//Prevent stale check
                move.setBrake(true);
            }

            checkStraight();

            if (deltaBack() < tile_size / 3) {
                move.setWheelTurn(-offsAngle);
                if (countDownMove1 <= 0 && speed < turnMaxSpeed / 3)
                    move.setEnginePower(-1.0);
            } else {
                if (countDownMove2 > 0) {
                    move.setBrake(true);
                    countDownMove2--;
                    tickTr = 0;//Prevent stale check
                }
                move.setWheelTurn(offsAngle);
                if (countDownMove2 <= 0) {
                    //System.out.println("Restore delegate after backward " + lastDelegate + " " + direction + " angle:" + self.getAngle());
                    backwardOn = true;
                    delegate = lastDelegate;
                    //delegate = new MoveForward();
                }
            }
            checkStale();
        }
    }

    void checkCanThrowProjectile() {
        if (self.getProjectileCount() > 0)
            if (checkCloseAngle(0, self.getHeight(), fireMaxDistance)) {
                move.setThrowProjectile(true);
                move.setThrowProjectile(true);
            }
    }

    boolean isCanUseNitro() {
        NextTurn t = findNextTurnFrom(cx, cy, direction, 4);
        if (null != t) {
            return (t.distanceTo() > tile_size * 4 && abs(getOffsetAngle()) < ANGLE_5);
        }
        return false;
    }

    void checkNitro() {
        if (!enable_nitro || null != currentTurn || brakeOn || self.getRemainingNitroTicks() > 0 || self.getNitroChargeCount() < 1
                || (null != ally && ally.getNitroChargeCount() == 0)) {
            return;
        }

        if (isCanUseNitro()) {
            //System.out.println(nextTurn.distanceTo());
            //System.out.println("Use nitro !!! " + nextTurn.distanceTo());
            move.setUseNitro(true);
        }
    }

    boolean checkCloseAngle(double angle, double spreadWidth, double distance) {
        for (Car car : world.getCars()) {
            if (!car.isTeammate()) {
                double d = self.getDistanceTo(car);
                if (d < distance && abs(abs(angle) - abs(self.getAngleTo(car))) < atan(spreadWidth / 2 / d)) {
                    return true;
                }
            } else if (car.getId() != self.getId()) {
                ally = car;
            }
        }
        return false;
    }

    boolean checkClose(double distance) {
        for (Car car : world.getCars()) {
            if (car.getId() != self.getId()) {
                if (self.getDistanceTo(car) < distance) {
                    return true;
                }
            }
        }
        return false;
    }

    void checkDistance() {
        if (null != ally) {
            double d = self.getDistanceTo(ally);
            double a = self.getAngleTo(ally);
            isForward = abs(a) > ANGLE_120 + ANGLE_30;
            isBackward = !isForward && d > self.getHeight();
            if ((isForward && d > 4 * tile_size)
                    || (isBackward && d < 2 * tile_size)) {
                move.setEnginePower(0.0);
            }
        }
    }

    public double getRightAngle(Direction direction) {
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

    double maxByAbs(double v1, double v2) {
        return abs(v1) > abs(v2) ? v1 : v2;
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
                    return self.getNextWaypointX() >= cx ? RIGHT : LEFT;
                }
            }
            if (nextTile == TOP_HEADED_T) {
                if (DOWN == dir) {
                    return self.getNextWaypointX() >= cx ? RIGHT : LEFT;
                }
            }
            if (nextTile == LEFT_HEADED_T) {
                if (RIGHT == dir) {
                    return self.getNextWaypointY() >= cy ? DOWN : UP;
                }
            }
            if (nextTile == RIGHT_HEADED_T) {
                if (LEFT == dir) {
                    return self.getNextWaypointY() >= cy ? DOWN : UP;
                }
            }

        }
        return direction;
    }

    static int[][] NEXT_TURN_DELTAS = {
            //X, Y
            {-1, 0},//L  
            {1, 0},//R
            {0, -1},//U
            {0, 1},//D
    };

    NextTurn findNextTurnFrom(int fromX, int fromY, Direction dir, int limit) {

        int dX = NEXT_TURN_DELTAS[dir.ordinal()][0];
        int dY = NEXT_TURN_DELTAS[dir.ordinal()][1];
        int maxX = world.getWidth();
        int maxY = world.getHeight();
        for (int x = fromX, y = fromY, c = 0; x < maxX && y < maxY && x > -1 && y > -1 && c <= limit; x = x + dX, y = y + dY, c++) {
            TileType tile = world.getTilesXY()[x][y];
            if (c > 0 && isTurnTile(tile, dir)) {
                return new NextTurn(x, y, dir);
            }
        }
        return null;
    }

    NextTurn findNextTurnFrom(NextTurn turn, int delta) {
        if (null != turn) {
            int ntX = turn.tileX;
            int ntY = turn.tileY;
            Direction ntD = turn.afterDirection;
            int[] xy = getNextTileXY(ntX, ntY, delta, ntD);
            TileType tileAfterTurn = world.getTilesXY()[xy[0]][xy[1]];
            if (isTurnTile(tileAfterTurn, ntD)) {
                NextTurn nt = new NextTurn(xy[0], xy[1], ntD);
                nt.deltaTile = delta;
                return nt;
            }
        }
        return delta < 2 ? findNextTurnFrom(turn, delta + 1) : null;
    }

    TileType getTile(double x, double y) {
        return world.getTilesXY()[getTileXY(x)][getTileXY(y)];
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

    double getEndSpeed(double dist) {
        if (0 == accel)
            return speed;
        return sqrt(speed * speed + 2 * dist * accel);
    }

    double quadro(double v1, double v2) {
        return abs(v1) * v1 / v2 * v2;
    }

    double cubic(double v1, double v2) {
        return v1 * v1 * v1 / v2 * v2 * v2;
    }

    double[] getTurnTileInsideCornerCoord(NextTurn nextTurn, double offset) {
        double x = nextTurn.x + TURN_CENTER_OFFSET_X[direction.ordinal()][nextTurn.afterDirection.ordinal()]
                * offset;
        double y = nextTurn.y + TURN_CENTER_OFFSET_Y[direction.ordinal()][nextTurn.afterDirection.ordinal()]
                * offset;
        return new double[] {x, y};
    }

    private static double getTileCenter(int tileXY) {
        return tile_size * tileXY + tile_size / 2;
    }

    private static int getTileXY(double tileXY) {
        return (int) (tileXY / tile_size);
    }

    void debugOffsets() {
        System.out.println(
                "Direction=" + direction
                        + ", centerX=" + centerX
                        + ", centerY=" + centerY
                        + ", lastCenterX=" + lastCenterX
                        + ", lastCenterY=" + lastCenterY
                        + ", offs. center=" + getOffsetCenter()
                        + ", offs. angle=" + getOffsetAngle()
                );
        if (null != currentTurn) {
            System.out.println(
                    "currentTurn=[" + currentTurn.tile
                            + ", " + currentTurn.afterDirection
                            + ", dist=" + currentTurn.dist
                            + ", cornerX=" + currentTurn.cornerX
                            + ", cornerY=" + currentTurn.cornerY
                            + ", turnDir=" + currentTurn.turnDirection
                            + "]"
                    );
        }
    }

    class PIDController {

        private double m_P; // factor for "proportional" control
        private double m_I; // factor for "integral" control
        private double m_D; // factor for "derivative" control
        private double m_input; // sensor input for pid controller
        private double m_maximumOutput = 1.0; // |maximum output|
        private double m_minimumOutput = -1.0; // |minimum output|
        private double m_maximumInput = 0.0; // maximum input - limit setpoint to this
        private double m_minimumInput = 0.0; // minimum input - limit setpoint to this
        private boolean m_continuous = false; // do the endpoints wrap around? eg. Absolute encoder
        private boolean m_enabled = false; //is the pid controller enabled
        private double m_prevError = 0.0; // the prior sensor input (used to compute velocity)
        private double m_totalError = 0.0; //the sum of the errors for use in the integral calc
        private double m_tolerance = 0.05; //the percetage error that is considered on target
        private double m_setpoint = 0.0;
        private double m_error = 0.0;
        private double m_result = 0.0;

        /**
         * Allocate a PID object with the given constants for P, I, D
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd) {
            setPID(Kp, Ki, Kd);

        }

        /**
         * Read the input, calculate the output accordingly, and write to the output.
         * This should only be called by the PIDTask
         * and is created during initialization.
         */
        private void calculate() {

            // If enabled then proceed into controller calculations
            if (m_enabled) {

                // Calculate the error signal
                m_error = m_setpoint - m_input;

                // !!!!DEBUG!!!
                //System.out.println(m_setpoint);

                // If continuous is set to true allow wrap around
                if (m_continuous) {
                    if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
                        if (m_error > 0) {
                            m_error = m_error - m_maximumInput + m_minimumInput;
                        } else {
                            m_error = m_error +
                                    m_maximumInput - m_minimumInput;
                        }
                    }
                }

                /* Integrate the errors as long as the upcoming integrator does
                   not exceed the minimum and maximum output thresholds */
                if (((m_totalError + m_error) * m_I < m_maximumOutput) &&
                        ((m_totalError + m_error) * m_I > m_minimumOutput)) {
                    m_totalError += m_error;
                }

                // Perform the primary PID calculation
                m_result = (m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError));

                // Set the current error to the previous error for the next cycle
                m_prevError = m_error;

                // Make sure the final result is within bounds
                if (m_result > m_maximumOutput) {
                    m_result = m_maximumOutput;
                } else if (m_result < m_minimumOutput) {
                    m_result = m_minimumOutput;
                }
            }
        }

        /**
         * Set the PID Controller gain parameters.
         * Set the proportional, integral, and differential coefficients.
         * @param p Proportional coefficient
         * @param i Integral coefficient
         * @param d Differential coefficient
         */
        public void setPID(double p, double i, double d) {
            m_P = p;
            m_I = i;
            m_D = d;
        }

        /**
         * Return the current PID result
         * This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID() {
            calculate();
            return m_result;
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         * @param continuous Set to true turns on continuous, false turns off continuous
         */
        public void setContinuous(boolean continuous) {
            m_continuous = continuous;
        }

        /**
         *  Set the PID controller to consider the input to be continuous,
         *  Rather then using the max and min in as constraints, it considers them to
         *  be the same point and automatically calculates the shortest route to
         *  the setpoint.
         */
        public void setContinuous() {
            this.setContinuous(true);
        }

        /**
         * Sets the maximum and minimum values expected from the input.
         *
         * @param minimumInput the minimum value expected from the input
         * @param maximumInput the maximum value expected from the output
         */
        public void setInputRange(double minimumInput, double maximumInput) {
            m_minimumInput = minimumInput;
            m_maximumInput = maximumInput;
            setSetpoint(m_setpoint);
        }

        /**
         * Sets the minimum and maximum values to write.
         *
         * @param minimumOutput the minimum value to write to the output
         * @param maximumOutput the maximum value to write to the output
         */
        public void setOutputRange(double minimumOutput, double maximumOutput) {
            m_minimumOutput = minimumOutput;
            m_maximumOutput = maximumOutput;
        }

        /**
         * Set the setpoint for the PIDController
         * @param setpoint the desired setpoint
         */
        public void setSetpoint(double setpoint) {
            if (m_maximumInput > m_minimumInput) {
                if (setpoint > m_maximumInput) {
                    m_setpoint = m_maximumInput;
                } else if (setpoint < m_minimumInput) {
                    m_setpoint = m_minimumInput;
                } else {
                    m_setpoint = setpoint;
                }
            } else {
                m_setpoint = setpoint;
            }
        }

        /**
         * Returns the current setpoint of the PIDController
         * @return the current setpoint
         */
        public double getSetpoint() {
            return m_setpoint;
        }

        /**
         * Retruns the current difference of the input from the setpoint
         * @return the current error
         */
        public double getError() {
            return m_error;
        }

        /**
         * Set the percentage error which is considered tolerable for use with
         * OnTarget. (Input of 15.0 = 15 percent)
         * @param percent error which is tolerable
         */
        public void setTolerance(double percent) {
            m_tolerance = percent;
        }

        /**
         * Return true if the error is within the percentage of the total input range,
         * determined by setTolerance. This asssumes that the maximum and minimum input
         * were set using setInput.
         * @return true if the error is less than the tolerance
         */
        public boolean onTarget() {
            return (abs(m_error) < m_tolerance / 100 *
                    (m_maximumInput - m_minimumInput));
        }

        /**
         * Begin running the PIDController
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Stop running the PIDController, this sets the output to zero before stopping.

         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Reset the previous error,, the integral term, and disable the controller.
         */
        public void reset() {
            disable();
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }

        public void setInput(double input) {
            m_input = input;
        }

    }

    public static void main(String[] args) {
        //        PIDController p = new PIDController(0.1, 0.0, 0.0);
        //        p.setPID(0.1, 0, 0);
        //        p.setContinuous();
        //        p.setInputRange(-180, 180);
        //        p.setOutputRange(-1, 1);
        //        p.setSetpoint(0);
        //        p.setTolerance(5);
        //        p.enable();
        //        p.setInput(0.0);
        //        double r = p.performPID();
        //        System.out.println(r);
    }
}
