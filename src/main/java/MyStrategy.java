import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.signum;
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
import model.BonusType;
import model.Car;
import model.Direction;
import model.Game;
import model.Move;
import model.TileType;
import model.Unit;
import model.World;

public final class MyStrategy implements Strategy {
    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_120 = 2 * PI / 3;
    private static final double ANGLE_90 = PI / 2;
    private static final double ANGLE_30 = PI / 6;
    private static final double ANGLE_15 = PI / 12;
    private static final double ANGLE_5 = PI / 36;

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
    static boolean enable_nitro = true;
    static boolean enable_turn_enter_offset = true;
    static boolean enable_bonus_lookup = false;
    static boolean enable_car_avoidance = true;
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
    private double fireMaxDistance;
    private double oilMaxDistance;
    private boolean isForward;
    private boolean isBackward;
    private boolean backwardOn;
    private int tickWait = 0;

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
        final Direction afterDirection;
        double dist;
        double turnDirection = 0;
        int deltaTile = 1;

        NextTurn(int tileX, int tileY, Direction currentDirection) {
            this.tileX = tileX;
            this.tileY = tileY;
            this.tile = world.getTilesXY()[tileX][tileY];
            this.x = getTileCenter(tileX);
            this.y = getTileCenter(tileY);
            this.afterDirection = resolveDirectionAfterTurn(this.tile, currentDirection);
            this.dist = null != self ? self.getDistanceTo(x, y) : 0;
            this.turnDirection = TURN_DIRECTION[currentDirection.ordinal()][afterDirection.ordinal()];
        }

        double distanceTo() {
            return dist;
        }
    }

    private NextTurn currentTurn;
    private NextTurn nextTurn;
    private NextTurn doubleTurn;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;
    private boolean brakeOn;

    private int cx;
    private int cy;

    private double centerX;
    private double centerY;
    private double lastCenterX;
    private double lastCenterY;
    private Direction direction;
    private Bonus bonus;

    private int turnCount = 0;
    private int tickTr = 0;
    private int staleMoveDelay;
    int tickCount;
    double speedAvg;

    private Delegate delegate;
    private NextTurn lastTurn;
    private TileType nextTile = EMPTY;

    //    

    static double A = 0.5;
    static double B = 4;
    static double C = 400;
    static double D = 1;
    static double E = 1;

    public MyStrategy() {
    }

    public MyStrategy(double a, double b, double c, double d, double e, double f, double g) {
        inside_turn_factor = a;
        outside_turn_factor = b;
        mobility_factor = c;
        mobility_2x_factor = d;
        stability_factor = e;
        turn_enter_offset_tiles = f;
        turn_max_speed = g;
    }

    double getTurnEnterOffset(NextTurn turn) {
        return speed * speed * A + speed * B + C;
    }

    double getTurnEnterSpeed() {
        if (null != getNextTurn()) {
            if (null != doubleTurn && nextTurn.distanceTo() < 1.5 * tile_size) {
                turnMaxSpeed = turn_max_speed;
                if ((doubleTurn.afterDirection != direction && 1 == doubleTurn.deltaTile)
                        || (doubleTurn.afterDirection == direction && 2 == doubleTurn.deltaTile)) {
                    turnMaxSpeed = D * turnMaxSpeed;
                }
                return E * turnMaxSpeed;
            }
        }
        return turnMaxSpeed;
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
            currentTurn = new NextTurn(cx, cy, direction);
            lastTurn = currentTurn;
            lastCenterX = currentTurn.x;
            lastCenterY = currentTurn.y;
        }
        centerX = lastCenterX;
        centerY = lastCenterY;

        maxSpeed = max_speed;
        mobility = mobility_factor;
        turnMaxSpeed = turn_max_speed;

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

        if (turning)
            return;

        int tileAhead = speed > drift_speed ? 2 : 1;

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

        if (null == lastTurn || nextTileXY[0] != lastTurn.tileX || nextTileXY[1] != lastTurn.tileY) {

            if (isTurnTile(nextTile, direction)) {
                if (null == nextTurn || nextTurn.tileX != nextTileXY[0] || nextTurn.tileY != nextTileXY[1]) {
                    nextTurn = new NextTurn(nextTileXY[0], nextTileXY[1], direction);
                    doubleTurn = findNextTurnFrom(nextTurn, 1);
                } else {
                    nextTurn.dist = self.getDistanceTo(nextTurn.x, nextTurn.y);
                }

                double dist = getNextTurn().distanceTo();
                double start = getTurnEnterOffset(getNextTurn());
                //System.out.println("start=" + start + ", dist=" + dist);
                if (dist < start) {
                    Direction dir = nextTurn.afterDirection;
                    if (null != dir && dir != direction) {
                        //System.out.println("Turn detected: " + direction + " -> " + dir + ", " + lastTurn.tile + " --- " + nextTile
                        //                                + ", start=" + start + ", dist=" + dist + ", speed=" + speed + ", tileAhead=" + tileAhead + ", turning=" + turning);

                        if (nextTurn.turnDirection != 0) {
                            delegate = new Turn(nextTurn);
                        }
                    }
                }
                if (delegate != old) {
                    lastTurn = nextTurn;
                }
            }

        }
        if (null == delegate)
            delegate = new MoveForward();
    }

    void checkStraight() {
        //        if (!turning) {
        double delta1 = stability_factor * speed / turnMaxSpeed * getOffsetAngle() / ANGLE_90;

        double offset = getOffsetCenter() / fromCenterToBorderWithWidth;
        //            //System.out.println("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

        double delta2 = mobility * OFFSET[direction.ordinal()] * offset;

        //        //System.out.println("CXY=" + lastCenterX + "," + lastCenterX + ", CenterXY=" + centerX + ", " + centerY + " Offs. center=" + getOffsetCenter()
        //                + " wheel correction 1="
        //                + delta2
        //                + " Offs. angle=" + getOffsetAngle() + " wheel correction 2=" + delta2);
        //            //System.out.println("Dir o1=" + delta1 + ", o2=" + delta2);

        delta1 += delta2;
        //            move.setEnginePower(0.2 + abs(ANGLE_90 - abs(offset)) / ANGLE_90);
        move.setWheelTurn(delta1);
        //        }
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
        if (enable_car_avoidance && !turning) {
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
        if (!backwardOn) {

            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            boolean increaseMobility = false;

            if (enable_turn_enter_offset && null != getNextTurn()) {
                if (nextTurn.distanceTo() < tile_size * 4) {
                    double m = outside_turn_factor;

                    if (null != doubleTurn) {
                        if (doubleTurn.afterDirection == direction) {
                            m = 0;
                        }
                        increaseMobility = true;
                    }

                    centerX = lastCenterX + m * fromCenterToBorderWithWidth * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().afterDirection.ordinal()];
                    centerY = lastCenterY + m * fromCenterToBorderWithWidth * TURN_OFFSET_OUT[direction.ordinal()][getNextTurn().afterDirection.ordinal()];
                }
            }

            pickBonus();

            if (increaseMobility) {
                mobility = mobility_factor * mobility_2x_factor;
            }
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
        if (enable_bonus_lookup && !turning) {
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

    double getOffsetCenter() {

        double offset = isHorizontal()
                ? self.getY() - centerY
                : self.getX() - centerX;
        return offset;
    }

    double getOffsetRealCenter() {

        double offset = isHorizontal()
                ? self.getY() - lastCenterY
                : self.getX() - lastCenterX;
        return offset;
    }

    void print() {
        int x = self.getNextWaypointX();
        int y = self.getNextWaypointY();
        //System.out.println(world.getTilesXY()[x][y]);
    }

    class MoveForward implements Delegate {
        MoveForward() {
            turning = false;
            print();
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            //System.out.println("Begin move forward");
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkBrake();
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
        double offsAngle = 0;

        MoveBackward(Delegate lastDelegate) {
            this.lastDelegate = lastDelegate;
            tickTr = 0;
            turning = false;
            moveBackBeginX = self.getX();
            moveBackBeginY = self.getY();
            //System.out.println("Begin move backward dir: " + direction + " angle:" + self.getAngle()
            //                    + " centerX=" + centerX + " centerY=" + centerY
            //                    + " cx=" + cx + " cy=" + cy
            //                    + " self.x=" + self.getX() + " self.y=" + self.getY());
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
        if (null != getNextTurn()) {
            return (nextTurn.distanceTo() > tile_size * 4 && abs(getOffsetAngle()) < ANGLE_5);
        }
        return false;
    }

    void checkNitro() {
        if (!enable_nitro || turning || brakeOn || self.getRemainingNitroTicks() > 0 || self.getNitroChargeCount() < 1
                || (null != ally && ally.getNitroChargeCount() == 0)) {
            return;
        }

        if (null != getNextTurn()) {
            if (isCanUseNitro()) {
                //System.out.println(nextTurn.distanceTo());
                //System.out.println("Use nitro !!! " + nextTurn.distanceTo());
                move.setUseNitro(true);
            }
        }
    }

    void checkBrake() {
        brakeOn = false;

        brakeOn = speed > getTurnEnterSpeed();
        if (brakeOn) {
            //            //System.out.println("Bake ON! " + speed);
            move.setEnginePower(0.3);
            move.setBrake(true);
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

    void checkMargins() {
        if (turning)
            return;
        double correction = 0.2;
        double dR = self.getX() + self.getWidth() / 2;
        double dL = self.getX() - self.getWidth() / 2;
        double dU = self.getY() - self.getWidth() / 2;
        double dD = self.getY() + self.getWidth() / 2;
        double mR = (cx + 1) * tile_size - game.getTrackTileMargin();
        double mL = cx * tile_size + game.getTrackTileMargin();
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
        //System.out.println("dR=" + dR + ", mR=" + mR + ", dL=" + dL + ", mL=" + mL + ", dU=" + dU + ", mU=" + mU + ", dD=" + dD + ", mD=" + mD);
    }

    class Turn implements Delegate {

        boolean hasCarInBack;
        boolean enteredTurnTile;
        double[] center;
        double endAngle = turn_end_delta_angle;
        double turnSpeedStart;
        double turnDir;

        Turn(NextTurn nextTurn) {
            turnDir = nextTurn.turnDirection;
            tickTr = 0;
            turnAngleBegin = self.getAngle();// getRightAngle();

            debugOffsets();
            hasCarInBack = self.getOilCanisterCount() > 0 && checkCloseAngle(ANGLE_180, 3 * tile_size, oilMaxDistance);
            currentTurn = nextTurn;
            currentTurn.turnDirection = get();
            center = getTurnTileInsideCornerCoord(currentTurn);
            turning = true;
            turningHalf = false;
            enteredTurnTile = cx == currentTurn.tileX && cy == currentTurn.tileY;
            //System.out.println("Start new turn " + (tile_size * cx + tile_size / 2) + "," + (tile_size * cy + tile_size / 2) + " center: " + center[0] + ", "
            //                            + center[1]);

            if (null != doubleTurn) {
                //                if (doubleTurn.deltaTile == 1)
                endAngle = doubleTurn.afterDirection == direction ? ANGLE_30 : ANGLE_120;
                //                else
                //                    endAngle = doubleTurn.direction == direction ? ANGLE_90 * 0.7 : ANGLE_120 * 0.7;
            }
            turnSpeedStart = turnMaxSpeed;
            //System.out.println("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + endAngle
            //                    + ", maxSpeed=" + maxSpeed
            //                    + ", max_speed=" + max_speed
            //                    + ", turnMaxSpeed=" + turnMaxSpeed
            //                    + ", turn_Max_Speed=" + turn_max_speed
            //                    + ", doubleTurn=" + doubleTurn
            //                    );
        }

        double get() {
            return turnDir;
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));

            //            if (null == doubleTurn)
            //                turnMaxSpeed = turnSpeedStart + 0.3 * turnSpeedStart * delta / ANGLE_90;

            checkBrake();

            boolean inside = cx == currentTurn.tileX && cy == currentTurn.tileY;

            // Установить новое направление
            if (!enteredTurnTile && inside) {
                enteredTurnTile = true;

            }

            if (enteredTurnTile) {
                direction = currentTurn.afterDirection;
            }

            double turnDelay = 0;

            //            if (enteredTurnTile && null == doubleTurn) {
            //                turnDelay = 0;//ANGLE_60 * 2 * OFFSET[direction.ordinal()] * getOffsetCenter() / tile_size;
            //            }

            if (isBackward
                    && turnCount > 1
                    && hasCarInBack
                    && enteredTurnTile) {
                move.setSpillOil(true);
                hasCarInBack = false;
            }

            if (delta >= endAngle / 2) {
                turnHalf(this);
            }

            if ((enteredTurnTile && delta >= endAngle + turnDelay) ||
                    (enteredTurnTile && !inside)) {
                //System.out.println("Turn finish: Angle is " + self.getAngle() + ", delta is " + delta);
                turnEnd(this);
            } else {
                //                double w = (turn_max_angular_speed - abs(self.getAngularSpeed())) / turn_max_angular_speed;
                //                (speed > drift_speed || inside ? 1 : 0.5)
                move.setWheelTurn(get());
                checkBrake();
            }
            if (speed < drift_speed)
                avoidCollisions(center[0], center[1]);
            //            checkStraight();
            checkCanThrowProjectile();
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

    NextTurn getNextTurn() {
        if (null == nextTurn) {
            nextTurn = findNextTurnFrom(cx, cy, direction, Integer.MAX_VALUE);

        }
        if (null == doubleTurn && null != nextTurn) {
            doubleTurn = findNextTurnFrom(nextTurn, 1);
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

    double distFromCenter(Unit unit) {
        double unitXY = isHorizontal() ? unit.getY() : unit.getX();
        double centerXY = isHorizontal() ? centerY : centerX;
        return unitXY - centerXY;
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
            //System.out.print("turnHalf: dir=" + direction);
            turningHalf = true;
            turnCount++;
        }
    }

    void turnEnd(Turn turn) {
        //System.out.println("turnEnd: dir=" + direction);
        nextTurn = null;
        lastCenterX = currentTurn.x;
        lastCenterY = currentTurn.y;
        currentTurn = null;
        doubleTurn = null;
        delegate = null;
        turning = false;

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

    double[] getTurnTileInsideCornerCoord(NextTurn nextTurn) {
        double x = tile_size * nextTurn.tileX + tile_size / 2 + TURN_CENTER_OFFSET_X[direction.ordinal()][nextTurn.afterDirection.ordinal()]
                * (tile_size / 2 - margins);
        double y = tile_size * nextTurn.tileY + tile_size / 2 + TURN_CENTER_OFFSET_Y[direction.ordinal()][nextTurn.afterDirection.ordinal()]
                * (tile_size / 2 - margins);
        return new double[] {x, y};
    }

    private static double getTileCenter(int tileXY) {
        return tile_size * tileXY + tile_size / 2;
    }

    private static int getTileXY(double tileXY) {
        return (int) (tileXY / tile_size);
    }

    void debugOffsets() {
        //System.out.println(
        //                "Direction=" + direction
        //                        + ", centerX=" + centerX
        //                        + ", centerY=" + centerY
        //                        + ", lastCenterX=" + lastCenterX
        //                        + ", lastCenterY=" + lastCenterY
        //                        + ", offs. center=" + getOffsetCenter()
        //                        + ", offs. angle=" + getOffsetAngle()
        //                );
        if (null != currentTurn) {
            //System.out.println(
            //                    "currentTurn=[" + currentTurn.tile
            //                            + ", " + currentTurn.afterDirection + "]"
            //                    );
        }
    }
}
