import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;
import static model.Direction.DOWN;
import static model.Direction.LEFT;
import static model.Direction.RIGHT;
import static model.Direction.UP;
import static model.TileType.BOTTOM_HEADED_T;
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

    static double mass_factor = 1.00 / 1250.00;

    static double inside_turn_factor = 0;//0..1
    static double outside_turn_factor = 0.5;//0..1
    static double mobility_factor = 0.00001;
    static double double_turn_mobility_increment = 5;
    static double double_turn_u_speed_decrement = 1.5;
    static double double_turn_z_speed_decrement = 0.8;
    static double stability_factor = 19.6;
    static double turn_max_speed = 18;
    static double turn_max_angular_speed = 0.033;
    static double turn_enter_offset_tiles = 1.2;
    static double drift_speed = 26;
    static boolean enable_nitro = true;
    static boolean enable_turn_enter_offset = true;
    static boolean enable_bonus_lookup = true;
    static boolean enable_car_avoidance = true;
    static boolean enable_corner_avoidance = true;
    static boolean enable_turn_enter_delay = false;
    static int stale_tick_count = 50;
    static double stale_speed_value = 0.1;
    static double stale_move_value = 2;
    static double max_speed = 151;
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

    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_120 = 2 * PI / 3.0;
    private static final double ANGLE_90 = PI / 2.0;
    private static final double ANGLE_20 = ANGLE_90 / 9 * 2;
    private static final double ANGLE_80 = ANGLE_90 / 9 * 8;
    private static final double ANGLE_60 = PI / 3.0;
    private static final double ANGLE_45 = PI / 4.0;
    private static final double ANGLE_30 = PI / 6.0;
    private static final double ANGLE_15 = PI / 12.0;
    private static final double ANGLE_5 = PI / 36.0;
    private static final double ANGLE_1 = PI / 180.0;

    private static double tile_size;
    double A = 0.01;
    double B = 0.05;
    double C = 1.3;

    private Car self;
    private Car ally;
    private World world;
    private Game game;
    private Move move;

    boolean enableCenter = false;
    boolean turning = false;

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
    private int serialTurnCount;

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
        final double cornerX;
        final double cornerY;
        final int tileX;
        final int tileY;
        final TileType tile;
        final Direction direction0;
        final Direction direction1;
        double distance;
        double turnDirection = 0;
        int deltaTile = 1;
        boolean entered = false;

        NextTurn(int tileX, int tileY, Direction currentDirection) {
            this.tileX = tileX;
            this.tileY = tileY;
            this.tile = world.getTilesXY()[tileX][tileY];
            this.x = getTileCenter(tileX);
            this.y = getTileCenter(tileY);
            this.direction0 = currentDirection;
            this.direction1 = resolveDirectionAfterTurn(this.tile, currentDirection);
            this.distance = null != self ? self.getDistanceTo(x, y) : 0;
            this.entered = this.x == cx && this.y == cy;
            this.turnDirection = TURN_DIRECTION[currentDirection.ordinal()][this.direction1.ordinal()];
            double[] c = getTurnTileInsideCornerCoord(this, tile_size / 2);
            this.cornerX = c[0];
            this.cornerY = c[1];
        }

        double distanceTo() {
            return distance;
        }

        double[] getTurnTileInsideCornerCoord(NextTurn turn, double offset) {
            double x = turn.x + TURN_CENTER_OFFSET_X[this.direction0.ordinal()][turn.direction1.ordinal()]
                    * offset;
            double y = turn.y + TURN_CENTER_OFFSET_Y[this.direction0.ordinal()][turn.direction1.ordinal()]
                    * offset;
            return new double[] {x, y};
        }

        @Override
        public String toString() {
            return tile + " " + x + ", " + y + " " + direction0 + " -> " + direction1 + " ";
        }
    }

    private NextTurn currentTurn;
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

    private Delegate delegate;
    private double turnEnterOffset = 0;

    double speedAvg;
    int tickCount;

    //    

    public MyStrategy(double a2, double b2, double c2) {
        A = a2;
        B = b2;
        C = c2;
    }

    public MyStrategy() {
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
            turn_max_speed = turn_max_speed / (mass_factor * self.getMass());
            drift_speed = drift_speed / (mass_factor * self.getMass());
            double_turn_mobility_increment = double_turn_mobility_increment / (mass_factor * self.getMass());

        }

        turnEnterOffset = tile_size * turn_enter_offset_tiles;

        cx = getTileXY(self.getX());
        cy = getTileXY(self.getY());

        if (null == direction) {
            direction = world.getStartingDirection();
            lastCenterX = getTileCenter(cx);
            lastCenterY = getTileCenter(cy);
        }
        centerX = lastCenterX;
        centerY = lastCenterY;

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

        defineCenter();

        if (null != currentTurn) {
            currentTurn.distance = self.getDistanceTo(currentTurn.x, currentTurn.y);
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
                tickWait = 0;
            }
        }

        lastX = self.getX();
        lastY = self.getY();

        lastSpeedX = self.getSpeedX();
        lastSpeedY = self.getSpeedY();

        //        log("X=" + self.getX() + ", Y=" + self.getY() + ", A=" + self.getAngle() + ", Vx=" + self.getSpeedX() + ", Vy=" + self.getSpeedY());
        if (speed > maxSpeed) {
            log("limit max speed " + (maxSpeed / speed));
            move.setEnginePower(maxSpeed / speed);
        }
        if (!self.isFinishedTrack() && world.getTick() > game.getInitialFreezeDurationTicks()) {
            tickCount++;
            speedAvg += speed;
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

        if ((deltaMove() < stale_move_value)
                && (speed < stale_speed_value)
                && (accel < stale_speed_value)
                && tickTr > stale_tick_count) {

            log("Detect stale : dXY=" + deltaMove() + ", dV=" + deltaSpeed() + " brake=" + move.isBrake() + ", power=" + move.getEnginePower());
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

    double getTurnDelay(double offset) {

        double offsetSide = isHorizontal()
                ? self.getY() - lastCenterX
                : self.getX() - lastCenterY;
        offsetSide = offsetSide * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
        return max(tile_size / 2, A * speed / turnMaxSpeed + B * offsetSide + C * tile_size);
    }

    void checkTurn() {

        if (turning)
            return;

        if (null == currentTurn) {
            currentTurn = findNextTurnFrom(cx, cy, direction, 5);
            if (null != currentTurn) {
                doubleTurn = findNextTurnFrom(currentTurn.tileX, currentTurn.tileY, currentTurn.direction1, 0);

            }
        }
        if (null != currentTurn) {
            double dist = currentTurn.distanceTo();

            double start = getTurnDelay(turnEnterOffset);
            //            if (speed > drift_speed) {
            //                start = speed / drift_speed * turnEnterOffset;
            //            } else 
            if (null != doubleTurn) {
                start = doubleTurn.direction1 != direction ? turnEnterOffset * 0.8 : turnEnterOffset * 0.8;
            }

            if (dist < start) {
                Direction dir = currentTurn.direction1;
                if (null != dir && dir != direction) {
                    log("Turn started: " + direction + " -> " + dir + ", --- " + currentTurn.tile
                            + ", start=" + start + ", dist=" + dist + ", speed=" + speed + ", tileAhead=" + 5 + ", turning=" + turning);
                    if (null != doubleTurn) {
                        log("DoubleTurn detected: " + doubleTurn);
                    }
                    double side = TURN_DIRECTION[direction.ordinal()][currentTurn.direction1.ordinal()];
                    if (side != 0.0) {
                        delegate = new Turn();
                    }
                }
            }
        }

        if (null == delegate)
            delegate = new MoveForward();
    }

    void checkStraight() {
        //        if (!turning) {
        double delta1 = stability_factor * speed / turnMaxSpeed * quadro(getOffsetAngle(), ANGLE_180);

        double offset = quadro(getOffsetCenter(), fromCenterToBorder);
        //            log("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

        double delta2 = mobility * offset;

        //        log("CXY=" + lastCenterX + "," + lastCenterX + ", CenterXY=" + centerX + ", " + centerY + " Offs. center=" + getOffsetCenter()
        //                + " wheel correction 1="
        //                + delta2
        //                + " Offs. angle=" + getOffsetAngle() + " wheel correction 2=" + delta2);
        //            log("Dir o1=" + delta1 + ", o2=" + delta2);

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
                double a2 = -a1 * OFFS_ANGLE_SIGN[direction.ordinal()][currentTurn.direction1.ordinal()];
                if (d1 < dist) {
                    if (a2 < -ANGLE_5) {
                        log("Avoid corner x=" + x + ", y=" + y + ", a=" + a1 + ", d=" + d1
                                + " self.x=" + self.getX()
                                + " self.y=" + self.getY()
                                + " correction=" + (-a1 * (angle - abs(a1)) / angle));
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

            log("Avoid car a1=" + a1 + ", a2=" + a2 + ", d1=" + d1 + ", d2=" + d2);
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

            maxSpeed = max_speed;
            mobility = mobility_factor;
            turnMaxSpeed = turn_max_speed;
            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            boolean increaseMobility = false;

            if (enable_turn_enter_offset && null != currentTurn) {
                double m = currentTurn.distanceTo() < tile_size && speed < turnMaxSpeed ? -inside_turn_factor : outside_turn_factor;

                if (speed > drift_speed) {
                    m = 1;
                }

                if (null != doubleTurn) {
                    if (doubleTurn.direction1 != direction) {
                        m = 1;
                    } else {
                        m = 0;
                    }
                    increaseMobility = true;
                }

                centerX = lastCenterX + m * fromCenterToBorder * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
                centerY = lastCenterY + m * fromCenterToBorder * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
            }

            pickBonus();

            if (increaseMobility) {
                mobility = mobility_factor * double_turn_mobility_increment;
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

    NextTurn getDoubleTurnAfter(NextTurn turn) {
        if (null != turn) {
            int ntX = turn.tileX;
            int ntY = turn.tileY;
            Direction ntD = turn.direction1;
            int[] xy = getNextTileXY(ntX, ntY, 1, ntD);
            TileType tileAfterTurn = world.getTilesXY()[xy[0]][xy[1]];
            if (isTurnTile(tileAfterTurn, ntD)) {
                return new NextTurn(xy[0], xy[1], ntD);
            }
        }
        return null;
    }

    double getOffsetCenter() {

        double offset = isHorizontal()
                ? self.getY() - centerY
                : self.getX() - centerX;
        return offset * OFFSET[direction.ordinal()];
    }

    class MoveForward implements Delegate {
        MoveForward() {
            turning = false;
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            log("Begin move forward");
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);
            checkTurn();
            checkBrake();
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
            log("Begin move backward dir: " + direction + " angle:" + self.getAngle()
                    + " centerX=" + centerX + " centerY=" + centerY
                    + " cx=" + cx + " cy=" + cy
                    + " self.x=" + self.getX() + " self.y=" + self.getY());
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
                    log("Restore delegate after backward " + lastDelegate + " " + direction + " angle:" + self.getAngle());
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
        if (null != currentTurn) {
            return (currentTurn.distanceTo() > tile_size * 4 && abs(getOffsetAngle()) < ANGLE_5);
        }
        return true;
    }

    void checkNitro() {
        if (!enable_nitro || turning || brakeOn || self.getRemainingNitroTicks() > 0 || self.getNitroChargeCount() < 1
                || (null != ally && ally.getNitroChargeCount() == 0)) {
            return;
        }

        if (isCanUseNitro()) {
            log("Use nitro !!! ");
            move.setUseNitro(true);
        }
    }

    void checkBrake() {
        brakeOn = false;

        if (turning || (null != currentTurn && currentTurn.distanceTo() < 2 * tile_size)) {
            if (null != doubleTurn) {
                if (doubleTurn.direction1 != direction) {
                    //                    maxSpeed = turn_max_speed / double_turn_u_speed_decrement;
                    turnMaxSpeed = turn_max_speed / double_turn_u_speed_decrement;
                } else {
                    maxSpeed = turn_max_speed;
                    turnMaxSpeed = turn_max_speed / double_turn_z_speed_decrement;
                }
            }
            brakeOn = speed > turnMaxSpeed;
        }
        if (brakeOn) {
            //            log("Bake ON! " + speed);
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

    class Turn implements Delegate {

        boolean hasCarInBack;
        boolean enteredTurnTile;
        double endAngle = ANGLE_1 * 70;
        double turnSpeedStart;
        double side;

        Turn() {
            serialTurnCount++;
            tickTr = 0;
            turnAngleBegin = getRightAngle();

            debugOffsets();
            hasCarInBack = self.getOilCanisterCount() > 0 && checkCloseAngle(ANGLE_180, 3 * tile_size, oilMaxDistance);
            side = currentTurn.turnDirection;
            turning = true;
            enteredTurnTile = cx == currentTurn.tileX && cy == currentTurn.tileY;
            log("--------------------------------------------------------------------------------------------------------------------");
            log("Start new turn " + currentTurn.x + "," + currentTurn.y + " corner: " + currentTurn.cornerX + ", " + currentTurn.cornerY
                    + ", center=" + centerX
                    + ", " + centerY
                    + ", self=" + self.getX()
                    + ", " + self.getY());

            if (null != doubleTurn) {
                endAngle = doubleTurn.direction1 == direction ? ANGLE_1 * 45 : ANGLE_120;
            }
            turnSpeedStart = turnMaxSpeed;
            log("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + endAngle
                    + ", maxSpeed=" + maxSpeed
                    + ", max_speed=" + max_speed
                    + ", turnMaxSpeed=" + turnMaxSpeed
                    + ", turn_Max_Speed=" + turn_max_speed
                    + ", doubleTurn=" + doubleTurn);
        }

        double get() {
            return side;
        }

        @Override
        public void move() {
            move.setEnginePower(1.0);

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));

            if (null == doubleTurn)
                turnMaxSpeed = turnSpeedStart + 0.3 * turnSpeedStart * delta / ANGLE_90;

            checkBrake();

            boolean inside = cx == currentTurn.tileX && cy == currentTurn.tileY;

            // Установить новое направление
            if (!enteredTurnTile && inside) {
                enteredTurnTile = true;

            }

            if (enteredTurnTile) {
                direction = currentTurn.direction1;
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

            if ((enteredTurnTile && delta >= endAngle + turnDelay) ||
                    (enteredTurnTile && !inside)) {
                log("Turn finish: Angle is " + self.getAngle() + ", delta is " + delta);
                log("--------------------------------------------------------------------------------------------------------------------");
                lastCenterX = currentTurn.x;
                lastCenterY = currentTurn.y;
                currentTurn = null;
                doubleTurn = null;
                delegate = null;
                turning = false;
                if (!inside && isTurnTile(world.getTilesXY()[cx][cy], direction)) {
                    currentTurn = new NextTurn(cx, cy, direction);
                }
            } else {
                //                double w = (turn_max_angular_speed - abs(self.getAngularSpeed())) / turn_max_angular_speed;
                //                (speed > drift_speed || inside ? 1 : 0.5)
                move.setWheelTurn(get());
                checkBrake();
            }
            if (speed < drift_speed)
                if (null != currentTurn) {
                    avoidCollisions(currentTurn.cornerX, currentTurn.cornerY);
                } else {
                    avoidCollisions(-1, -1);
                }
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
        for (int x = fromX + dX, y = fromY + dY, c = 0; x < maxX && y < maxY && x > -1 && y > -1 && c <= limit; x = x + dX, y = y + dY, c++) {
            TileType tile = world.getTilesXY()[x][y];
            if (isTurnTile(tile, dir)) {
                return new NextTurn(x, y, dir);
            }
        }
        return null;
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

    private static double getTileCenter(int tileXY) {
        return tile_size * tileXY + tile_size / 2;
    }

    private static int getTileXY(double tileXY) {
        return (int) (tileXY / tile_size);
    }

    void debugOffsets() {
        log("Direction=" + direction
                + ", centerX=" + centerX
                + ", centerY=" + centerY
                + ", lastCenterX=" + lastCenterX
                + ", lastCenterY=" + lastCenterY
                + ", offs. center=" + getOffsetCenter()
                + ", offs. angle=" + getOffsetAngle());
        if (null != currentTurn) {
            log("currentTurn=[" + currentTurn.tile
                    + ", " + currentTurn.direction1 + "]");
        }
    }

    void debugTurn(NextTurn turn) {
        log("Turn  " + direction + " -> " + turn.direction1 + ", " + " --- " + turn.tile
                + ", dist=" + turn.distanceTo() + ", speed=" + speed);
    }

    public static void main(String[] args) {

        int cx = 0;
        int cy = 2;

        Direction d1 = LEFT;
        Direction d2 = UP;

        tile_size = 800;

        double x = getTileCenter(cx) + TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]
                * (tile_size / 2 - 80);
        double y = tile_size * cy + tile_size / 2 + TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]
                * (tile_size / 2 - 80);

        log(TURN_CENTER_OFFSET_X[d1.ordinal()][d2.ordinal()]);
        log(TURN_CENTER_OFFSET_Y[d1.ordinal()][d2.ordinal()]);

        log(x);
        log(y);
    }

    static void log(Object str) {
        //
    }
}
