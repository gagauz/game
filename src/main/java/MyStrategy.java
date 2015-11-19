import model.*;

import java.util.*;

import static java.lang.Math.*;
import static model.Direction.*;
import static model.TileType.*;

public final class MyStrategy implements Strategy {

    static double mass_factor = 1.00 / 1250.00;

    static double inside_turn_factor = 0;//0..1
    static double outside_turn_factor = 0.5;//0..1
    static double mobility_factor = 0.000005;
    static double double_turn_mobility_increment = 5;
    static double double_turn_u_speed_decrement = 1.8;
    static double double_turn_z_speed_decrement = 1.0;
    static double double_turn_z_end_angle = 40;
    static double double_turn_u_end_angle = 90;
    static double stability_factor = 20;
    static double turn_max_speed = 18;
    static double turn_max_angular_speed = 0.033;
    static double turn_enter_offset_tiles = 0;
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
    double A = 0.0;
    double B = 0.06;
    double C = 1.36;
    double D = 0.9;
    double E = 0.9;

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

    private boolean[][] untouchedWaypoints;
    private boolean[][] cleared;

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
        boolean turning;

        NextTurn(int tileX, int tileY, Direction currentDirection, Direction newDirection) {
            this.tileX = tileX;
            this.tileY = tileY;
            this.tile = world.getTilesXY()[tileX][tileY];
            this.x = getTileCenter(tileX);
            this.y = getTileCenter(tileY);
            this.direction0 = currentDirection;
            this.direction1 = newDirection;
            this.distance = null != self ? self.getDistanceTo(x, y) : 0;
            this.entered = this.x == cx && this.y == cy;
            this.turnDirection = TURN_DIRECTION[currentDirection.ordinal()][this.direction1.ordinal()];
            double[] c = getTurnTileInsideCornerCoord(this, tile_size / 2 + margins);
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
            return tile + " [" + tileX + "," + tileY + "] " + direction0 + " -> " + direction1;
        }
    }

    private NextTurn currentTurn;
    private NextTurn lastTurn;
    private NextTurn doubleTurn;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;
    private boolean brakeOn;

    private int cx;
    private int cy;

    private double centerXY;
    private double lastCenterXY;
    private Direction direction;
    private Bonus bonus;

    private int tickTr = 0;
    private int staleMoveDelay;

    private Delegate delegate;
    private double turnEnterOffset = 0;
    private double insideTurnFactor = 0;

    double speedAvg;
    int tickCount;
    int wayPointIndex;
    int wayPointX;
    int wayPointY;

    //    

    public MyStrategy(double a2, double b2, double c2, double d2, double e2) {
        this();
        A = a2;
        B = b2;
        C = c2;
        D = d2;
        E = e2;
    }

    public MyStrategy() {
        turn_enter_offset_tiles = C;
        inside_turn_factor = D;
        outside_turn_factor = E;
    }

    int[] getNextWayPointXYFromIndex(int fromIndex) {
        int index = fromIndex < world.getWaypoints().length - 1 ? fromIndex + 1 : 0;
        return world.getWaypoints()[index];
    }

    void initWaypoints() {
        untouchedWaypoints = new boolean[world.getWidth()][world.getHeight()];
        for (int i = 0; i < world.getWaypoints().length; i++) {
            int x = world.getWaypoints()[i][0];
            int y = world.getWaypoints()[i][1];
            untouchedWaypoints[x][y] = true;
        }
    }

    void init() {
        direction = world.getStartingDirection();
        lastCenterXY = getTileCenter(getCenter(cx, cy));
        wayPointIndex = self.getNextWaypointIndex();
        wayPointX = self.getNextWaypointX();
        wayPointY = self.getNextWaypointY();
        insideTurnFactor = inside_turn_factor;

        cleared = new boolean[world.getWidth()][world.getHeight()];
        startX = getTileXY(self.getX());
        startY = getTileXY(self.getY());

        initWaypoints();

        //        System.out.println("{");
        //        for (int i = 0; i < world.getTilesXY().length; i++) {
        //            System.out.print("{");
        //            for (int j = 0; j < world.getTilesXY()[i].length; j++) {
        //                if (j > 0)
        //                    System.out.print(", ");
        //                System.out.print(world.getTilesXY()[i][j]);
        //
        //            }
        //            System.out.println("},");
        //        }
        //        System.out.println("}");
    }

    int startX = -1;
    int startY = -1;

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
            init();
        } else {
            if (startX == cx && startY == cy) {
                cleared = new boolean[world.getWidth()][world.getHeight()];
            } else {
                cleared[cx][cy] = true;
            }
        }
        centerXY = lastCenterXY;

        if (cx == wayPointX && cy == wayPointY) {
            log("Waypoint [" + cx + "," + cy + "] cleared.");
            wayPointIndex = self.getNextWaypointIndex();
            if (wayPointIndex == 1) {
                initWaypoints();
            }
            untouchedWaypoints[cx][cy] = false;
            wayPointX = world.getWaypoints()[wayPointIndex][0];
            wayPointY = world.getWaypoints()[wayPointIndex][1];

        }

        double speed0 = sqrt(pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2));
        accel = speed0 - speed;
        speed = speed0;

        defineCenter();

        if (null != currentTurn) {
            currentTurn.distance = self.getDistanceTo(currentTurn.x, currentTurn.y);
        }

        if (null != pathFinder) {
            pathFinder.distance = self.getDistanceTo(getTileCenter(pathFinder.x), getTileCenter(pathFinder.y));
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

        if (null != pathFinder && pathFinder.x == cx && pathFinder.y == cy && pathFinder.direction == direction) {
            pathFinder = null;
        }

        //        log("X=" + self.getX() + ", Y=" + self.getY() + ", A=" + self.getAngle() + ", Vx=" + self.getSpeedX() + ", Vy=" + self.getSpeedY());
        //        if (speed > maxSpeed) {
        //            log("limit max speed " + (maxSpeed / speed));
        //            move.setEnginePower(maxSpeed / speed);
        //        }
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

    double getTurnDelay() {

        double offsetSide = getCenter(self.getX(), self.getY()) - lastCenterXY;
        offsetSide = offsetSide * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
        return max(tile_size / 2, A * speed / turnMaxSpeed + B * offsetSide + C * tile_size);
    }

    PathFinder pathFinder;

    void checkTurn() {

        if (turning)
            return;

        debugOffsets();

        if (null == pathFinder) {
            int fx = cx;//null == lastTurn ? cx : lastTurn.tileX;
            int fy = cy;//null == lastTurn ? cy : lastTurn.tileY;
            pathFinder = getDirectionForClosestWayToTheNextWaypoint(fx, fy, direction, wayPointIndex, Integer.MAX_VALUE);
        }

        if (null == currentTurn && pathFinder != null) {

            PathFinder path = pathFinder;

            if (null != path) {
                while (null != path && path.direction == direction) {
                    log("Replace " + pathFinder + " with " + path.child);
                    path = path.child;
                }
            }
            if (null != path) {
                currentTurn = new NextTurn(path.x, path.y, direction, path.direction);
                pathFinder = pathFinder.child;

                //            currentTurn = findNextTurnFrom(x, y, direction, Integer.MAX_VALUE);
                //                doubleTurn = findNextTurnFrom(currentTurn.tileX, currentTurn.tileY, currentTurn.direction1, 1);
                log("Next turn detected: " + currentTurn + " double : " + doubleTurn);
            }
        }
        if (null != currentTurn) {
            double dist = currentTurn.distanceTo();

            double start = getTurnDelay();
            if (speed > drift_speed) {
                start = speed / drift_speed * turnEnterOffset;
            } else if (null != doubleTurn) {
                start = doubleTurn.direction1 != direction ? turnEnterOffset * D : turnEnterOffset * E;
            }

            if (dist < start) {
                Direction dir = currentTurn.direction1;
                if (null != dir && dir != direction) {
                    double side = TURN_DIRECTION[direction.ordinal()][currentTurn.direction1.ordinal()];
                    if (side != 0.0) {
                        log("--------------------------------------------------------------------------------------------------------------------");
                        log(currentTurn.direction0 + " -> " + currentTurn.direction1 + " " + currentTurn.tile + " ["
                                + currentTurn.tileX + ","
                                + currentTurn.tileY + "] "
                                + currentTurn.turnDirection
                                + ", start=" + start + ", dist=" + dist + ", speed=" + speed);
                        if (null != doubleTurn) {
                            log("DoubleTurn detected: " + doubleTurn);
                        }
                        log("Start new turn " + currentTurn.x + "," + currentTurn.y + " corner: " + currentTurn.cornerX + ", " + currentTurn.cornerY
                                + ", center=" + centerXY);

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

    double getClosestPoint(Car unit, double[] forPoint, double dist) {
        double a = unit.getAngle();
        double b = atan(unit.getWidth() / unit.getHeight());
        double d2 = pow(unit.getWidth() / 2, 2) + pow(unit.getHeight() / 2, 2);
        double[][] points = new double[4][2];
        points[0][0] = unit.getX() + d2 * cos(a + b);
        points[1][0] = unit.getX() + d2 * cos(a - b);
        points[2][0] = unit.getX() - d2 * cos(a + b);
        points[3][0] = unit.getX() - d2 * cos(a - b);

        points[0][1] = unit.getY() + d2 * sin(a + b);
        points[1][1] = unit.getY() + d2 * sin(a - b);
        points[2][1] = unit.getY() - d2 * sin(a + b);
        points[3][1] = unit.getY() - d2 * sin(a - b);
        d2 = dist;
        double d1 = 0;
        for (int i = 0; i < 4; i++) {
            d1 = self.getDistanceTo(points[i][0], points[i][1]);
            if (d1 < d2) {
                d2 = d1;
                forPoint[0] = points[i][0];
                forPoint[1] = points[i][0];
            }
        }
        return d2;
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
                    if (a2 < 0) {
                        log("Avoid corner x=" + x + ", y=" + y + ", a=" + a1 + ", d=" + d1
                                + " self.x=" + self.getX()
                                + " self.y=" + self.getY()
                                + " correction=" + (-a1 * (angle - abs(a1)) / angle));
                        a1 = signum(a1);
                        dist = d1;
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
                    if (abs(a) < ANGLE_30 && abs(tan(a) * d2) < car.getWidth() / 2) {
                        carToAvoid = car;
                        dist = d2;
                    }
                }
            }
        }
        if (null != carToAvoid) {
            double a2 = self.getAngleTo(carToAvoid);
            a1 = a1 * d1 / d2;
            a2 = a2 * (angle - abs(a2)) / angle * d2 / d1;

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
                double m = currentTurn.distanceTo() < tile_size && speed < turnMaxSpeed ? -insideTurnFactor : outside_turn_factor;

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

                centerXY = lastCenterXY + m * fromCenterToBorder * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
            }

            pickBonus();

            if (increaseMobility) {
                mobility = mobility_factor * double_turn_mobility_increment;
            }
            //Check limits
            double deltaCXY = lastCenterXY - centerXY;
            if (abs(deltaCXY) > fromCenterToBorderWithWidth) {
                centerXY = lastCenterXY + signum(centerXY - lastCenterXY) * fromCenterToBorderWithWidth;
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
                            double dxy = getCenter(self.getX(), self.getY()) - getCenter(b.getX(), b.getY());
                            if (abs(dxy) < fromCenterToBorder) {
                                int x = getTileXY(b.getX());
                                int y = getTileXY(b.getY());
                                boolean turn = null != currentTurn && currentTurn.tileX == x && currentTurn.tileY == y;
                                if (!turn) {
                                    closestXY = dist;
                                    bonus = b;
                                }
                            }
                        }
                    }
                }

            } else {

                if ((isUp() && self.getY() < bonus.getY())
                        || (isDown() && self.getY() > bonus.getY())
                        || (isLeft() && self.getX() < bonus.getX())
                        || (isRight() && self.getX() > bonus.getX())) {
                    bonus = null;
                } else {
                    centerXY = getCenter(bonus.getX(), bonus.getY())/* - bonus.getWidth() / 2 * signum(bonus.getX() - lastCenterX)*/;
                }
            }
        }
    }

    double getOffsetCenter() {
        double offset = getCenter(self.getX(), self.getY()) - centerXY;
        return offset * OFFSET[direction.ordinal()];
    }

    double getOffsetRealCenter() {
        double offset = getCenter(self.getX(), self.getY()) - lastCenterXY;
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
                    + " center=" + centerXY
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
        if (self.getProjectileCount() > 0) {
            if (checkCloseAngle(0, self.getHeight(), fireMaxDistance)) {
                move.setThrowProjectile(true);
            }
        }
        if (self.getOilCanisterCount() > 0) {
            if (null != currentTurn && currentTurn.distance < speed / turnMaxSpeed * tile_size) {
                if (checkCloseAngle(ANGLE_180, tile_size, oilMaxDistance)) {
                    move.setSpillOil(true);
                }
            }
        }
    }

    boolean isCanUseNitro() {
        boolean b = abs(abs(getRightAngle()) - abs(self.getAngle())) < ANGLE_1 * 5;
        if (null != currentTurn) {
            b = b && currentTurn.distance > tile_size * 4;
        } else if (null != pathFinder) {
            b = b && pathFinder.direction == direction && pathFinder.distance > tile_size * 3;
        }
        return b;
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

        boolean enteredTurnTile;
        double endAngle = ANGLE_1 * 60;
        double turnSpeedStart;
        double side;

        Turn() {
            currentTurn.turning = true;
            tickTr = 0;
            turnAngleBegin = getRightAngle();
            debugOffsets();
            side = currentTurn.turnDirection;
            turning = true;
            enteredTurnTile = cx == currentTurn.tileX && cy == currentTurn.tileY;
            if (null != doubleTurn) {
                endAngle = doubleTurn.direction1 == direction
                        ? double_turn_z_end_angle * ANGLE_1
                        : double_turn_u_end_angle * ANGLE_1;
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
            move.setEnginePower(0.5);

            double delta = abs(abs(self.getAngle()) - abs(turnAngleBegin));

            if (null == doubleTurn)
                turnMaxSpeed = turnSpeedStart + 0.3 * turnSpeedStart * delta / endAngle;

            boolean inside = cx == currentTurn.tileX && cy == currentTurn.tileY;

            // Установить новое направление
            if (!enteredTurnTile && inside) {
                enteredTurnTile = true;
                direction = currentTurn.direction1;
                pathFinder = getDirectionForClosestWayToTheNextWaypoint(currentTurn.tileX, currentTurn.tileY, direction, wayPointIndex, Integer.MAX_VALUE);
            }

            //            if (enteredTurnTile && null == doubleTurn) {
            //                turnDelay = 0;//ANGLE_60 * 2 * OFFSET[direction.ordinal()] * getOffsetCenter() / tile_size;
            //            }

            if ((enteredTurnTile && delta >= endAngle) ||
                    (enteredTurnTile && !inside)) {
                log("Turn finished: Angle is " + self.getAngle() + ", delta is " + delta);
                debugOffsets();
                log("--------------------------------------------------------------------------------------------------------------------");
                lastCenterXY = getCenter(currentTurn.x, currentTurn.y);
                lastTurn = currentTurn;
                currentTurn = null;
                doubleTurn = null;
                delegate = null;
                turning = false;
                insideTurnFactor = inside_turn_factor;
                return;
            } else {
                //                double w = (turn_max_angular_speed - abs(self.getAngularSpeed())) / turn_max_angular_speed;
                //                (speed > drift_speed || inside ? 1 : 0.5)
                move.setWheelTurn(get() * (endAngle - delta) / endAngle);
                checkBrake();
            }
            if (speed < drift_speed)
                avoidCollisions(currentTurn.cornerX, currentTurn.cornerY);
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

    Direction[][][] DIRECTION_MATRIX = new Direction[][][] {
            //E     V      H        LTC      RTC      LBC      RBC     LHT           RHT            THT            BHT            C        U
            {null, null, {LEFT}, {DOWN}, null, {UP}, null, null, {UP, DOWN}, {LEFT, UP}, {LEFT, DOWN}, {LEFT, UP, DOWN}, {LEFT}},//L
            {null, null, {RIGHT}, null, {DOWN}, null, {UP}, {UP, DOWN}, null, {RIGHT, UP}, {RIGHT, DOWN}, {RIGHT, UP, DOWN}, {RIGHT}},//R
            {null, {UP}, null, {RIGHT}, {LEFT}, null, null, {UP, LEFT}, {UP, RIGHT}, null, {LEFT, RIGHT}, {UP, LEFT, RIGHT}, {UP}},//U
            {null, {DOWN}, null, null, null, {RIGHT}, {LEFT}, {DOWN, LEFT}, {DOWN, RIGHT}, {LEFT, RIGHT}, null, {DOWN, LEFT, RIGHT}, {DOWN}}//D
    };

    static int branchIndex = 0;

    class PathFinder implements Comparable<PathFinder> {

        double distance;
        PathFinder child;
        PathFinder parent;
        Direction direction;
        int turnCount = 0;
        int x;
        int y;
        int wpx;
        int wpy;
        int branch;
        public boolean waypoint;
        public boolean invalid;
        public boolean exited;
        public boolean cycle;

        // For debug
        PathFinder(Direction dir, PathFinder par, int x, int y, int[] wp) {
            this.branch = ++branchIndex;
            this.direction = dir;
            this.parent = par;
            if (par != null) {
                if (dir != par.direction)
                    par.turnCount++;
                turnCount = par.turnCount;
            }
            this.x = x;
            this.y = y;
            this.wpx = wp[0];
            this.wpy = wp[1];
            System.out.println("New " + this);
        }

        int getTurnCount() {
            return turnCount;
        }

        void selectChild(PathFinder child) {
            this.turnCount = child.turnCount;
            if (child.invalid) {
                this.invalid = true;
            }
            if (child.cycle) {
                this.cycle = true;
            }
            if (child.exited) {
                this.exited = true;
            }
            if (child.waypoint) {
                this.waypoint = true;
            }
            this.child = child;
        }

        public PathFinder selectChild(List<PathFinder> children) {
            Collections.sort(children);
            for (PathFinder child : children) {
                if (!child.exited) {
                    selectChild(child);
                    return child;
                }
            }
            return null;
        }

        @Override
        public int compareTo(PathFinder o) {
            if (waypoint != o.waypoint) {
                return waypoint ? -1 : 1;
            }
            if (invalid != o.invalid) {
                return invalid ? 1 : -1;
            }
            if (cycle != o.cycle) {
                return cycle ? 1 : -1;
            }
            if (turnCount != o.turnCount) {
                return turnCount - o.turnCount;
            }

            return 0;
        }

        @Override
        public String toString() {
            return "Way <" + (null != parent ? parent.branch + "->" : "") + branch + ", " + direction + " [" + x + "," + y + "] count=" + turnCount
                    + ", invalid=" + invalid
                    + ", exited=" + exited
                    + ", cycled=" + cycle
                    + ", waypoint=" + waypoint + ">";
        }

    }

    Direction[] sortDirections(final Direction[] array, final int x, final int y, final int[] waypoint) {
        Comparator<Direction> c = new Comparator<Direction>() {
            @Override
            public int compare(Direction o1, Direction o2) {
                if (LEFT == o1 && RIGHT == o2) {
                    if (LEFT == o1) {
                        return waypoint[0] - x;
                    } else {
                        return x - waypoint[0];
                    }
                } else if (UP == o1 && DOWN == o2) {
                    if (UP == o1) {
                        return waypoint[1] - y;
                    } else {
                        return y - waypoint[1];
                    }
                }
                return 0;
            }
        };
        Direction[] array2 = new Direction[array.length];
        System.arraycopy(array, 0, array2, 0, array.length);
        Arrays.sort(array2, c);
        return array2;
    }

    void resolveClosestPathToWaypoint(int fromX, int fromY, int waypointIndex, PathFinder pathFinder, boolean[][] visitedTiles) {

        System.out.println("Enter: " + pathFinder.direction + " [" + fromX + "," + fromY + "], waypointIndex=" + waypointIndex + " ["
                + world.getWaypoints()[waypointIndex][0]
                + "," + world.getWaypoints()[waypointIndex][1] + "]");

        //Limit lookup
        if (pathFinder.getTurnCount() > 6) {
            pathFinder.invalid = true;
            log("Exit: " + pathFinder + " limit reached for " + pathFinder);
            return;
        }

        //        if (visited[fromX][fromY]) {
        //            path.invalid = true;
        //            System.out.println("Exit: This cell is already visited " + path);
        //            return;
        //        }

        if (cleared[fromX][fromY]) {
            pathFinder.cycle = true;
        }

        final int[] nextWaypoints = world.getWaypoints()[waypointIndex];
        final Direction fromDir = pathFinder.direction;

        int dX = NEXT_TURN_DELTAS[fromDir.ordinal()][0];
        int dY = NEXT_TURN_DELTAS[fromDir.ordinal()][1];
        int maxX = world.getWidth();
        int maxY = world.getHeight();
        for (int x = fromX + dX, y = fromY + dY; x < maxX && y < maxY && x > -1 && y > -1; x = x + dX, y = y + dY) {

            final TileType tile = world.getTilesXY()[x][y];

            final Direction[] directions = DIRECTION_MATRIX[fromDir.ordinal()][tile.ordinal()];

            if (null == directions) {
                break;
            }

            if (x == nextWaypoints[0] && y == nextWaypoints[1]) {
                //If fouded at least 2 waypoints, stop
                pathFinder.waypoint = true;
                log("Exit: " + pathFinder + " Waypoint founded, exit this way");
                return;
            } else if (untouchedWaypoints[x][y]) {
                // Exclude path that leads to waypoints out off order
                //                pathFinder.exited = true;
                //                log("Exit: " + pathFinder + " Invalid waypoint founded, exit this way");
                //                return;
            }

            if (directions.length == 1 && fromDir == directions[0]) {
                continue;
            }

            if (!visitedTiles[x][y]) {
                visitedTiles[x][y] = true;

                if (null != directions) {
                    log("Directions [" + x + "," + y + "] " + Arrays.asList(directions));
                    if (directions.length == 1) {
                        PathFinder way = new PathFinder(directions[0], pathFinder, x, y, nextWaypoints);
                        resolveClosestPathToWaypoint(x, y, waypointIndex, way, visitedTiles);
                        pathFinder.selectChild(way);
                        log("Exit: " + pathFinder + " with single child " + way);
                        return; //?
                    } else {
                        //                        Direction[] directions2 = sortDirections(directions, x, y, nextWaypoints);
                        List<PathFinder> ways = new ArrayList<>(directions.length);
                        for (Direction d : directions) {
                            PathFinder finder = new PathFinder(d, pathFinder, x, y, nextWaypoints);
                            resolveClosestPathToWaypoint(x, y, waypointIndex, finder, visitedTiles);
                            ways.add(finder);
                        }
                        PathFinder way = pathFinder.selectChild(ways);
                        log("Exit: " + pathFinder + " with selected child " + way);
                        return; //?
                    }

                }
            } else {
                log("Skip: [" + x + "," + y + "] is visited");
                //resolver.invalid = true;
                //return;
            }
        }
        pathFinder.exited = true;
        log("Exit: " + pathFinder + " No suitable directions");
    }

    PathFinder getDirectionForClosestWayToTheNextWaypoint(int fromX, int fromY, Direction dir, int waypointIndex, int limit) {

        branchIndex = 0;
        int[] nextWaypoints = world.getWaypoints()[waypointIndex];

        log("Enter: " + dir + " [" + fromX + "," + fromY + "], waypointIndex=" + waypointIndex + " ["
                + world.getWaypoints()[waypointIndex][0]
                + "," + world.getWaypoints()[waypointIndex][1] + "]");

        int dX = NEXT_TURN_DELTAS[dir.ordinal()][0];
        int dY = NEXT_TURN_DELTAS[dir.ordinal()][1];
        int maxX = world.getWidth();
        int maxY = world.getHeight();

        boolean[][] visitedTiles = new boolean[world.getWidth()][world.getHeight()];
        visitedTiles[fromX][fromY] = true;

        for (int x = fromX + dX, y = fromY + dY, c = 0; x < maxX && y < maxY && x > -1 && y > -1 && c < limit; x = x + dX, y = y + dY, c++) {

            final TileType tile = world.getTilesXY()[x][y];
            log("Tile [" + x + "," + y + "] " + tile);
            Direction[] directions = DIRECTION_MATRIX[dir.ordinal()][tile.ordinal()];

            if (null == directions) {
                break;
            }

            visitedTiles[x][y] = true;

            if (directions.length == 1 && dir == directions[0]) {
                continue;
            }

            if (x == nextWaypoints[0] && y == nextWaypoints[1]) {
                waypointIndex++;
                if (waypointIndex == world.getWaypoints().length) {
                    waypointIndex = 0;
                }
                nextWaypoints = world.getWaypoints()[waypointIndex];
                log("Enter waypoint, increment. next is " + waypointIndex);
            }

            log("Directions [" + x + "," + y + "] " + Arrays.asList(directions));
            if (directions.length == 1) {
                System.out.println("Exit: single way is " + directions[0]);
                return new PathFinder(directions[0], null, x, y, nextWaypoints);
            }
            List<PathFinder> ways = new ArrayList<>(directions.length);
            for (Direction d : directions) {
                PathFinder finder = new PathFinder(d, null, x, y, nextWaypoints);
                resolveClosestPathToWaypoint(x, y, waypointIndex, finder, visitedTiles);
                if (!finder.exited) {
                    ways.add(finder);
                }
            }
            if (!ways.isEmpty()) {
                Collections.sort(ways);
                log("Exit: closest way is " + ways.get(0));
                return ways.get(0);
            }
        }
        log("Exit: No suitable directions");
        return null;
    }

    static int[][] NEXT_TURN_DELTAS = {
            //X, Y
            {-1, 0},//L  
            {1, 0},//R
            {0, -1},//U
            {0, 1},//D
    };

    //    NextTurn findNextTurnFrom(int fromX, int fromY, Direction dir, int limit) {
    //
    //        PathFinder way = getDirectionForClosestWayToTheNextWaypoint(fromX, fromY, dir, wayPointIndex, limit);
    //        if (null != way) {
    //            return new NextTurn(way.x, way.y, dir, way.direction);
    //        }
    //        return null;
    //    }

    TileType getTile(double x, double y) {
        return world.getTilesXY()[getTileXY(x)][getTileXY(y)];
    }

    double distFromCenter(Unit unit) {
        double unitXY = getCenter(unit.getX(), unit.getY());
        return unitXY - centerXY;
    }

    boolean isTurnTileType(int x, int y) {
        TileType type = world.getTilesXY()[x][y];
        return (LEFT_TOP_CORNER == type)
                || (LEFT_BOTTOM_CORNER == type)
                || (RIGHT_BOTTOM_CORNER == type)
                || (RIGHT_TOP_CORNER == type)
                || (LEFT_HEADED_T == type)
                || (RIGHT_HEADED_T == type)
                || (TOP_HEADED_T == type)
                || (BOTTOM_HEADED_T == type)
                || (CROSSROADS == type);
    }

    boolean isAlwaysTurnTileType(int x, int y) {
        TileType type = world.getTilesXY()[x][y];
        return (LEFT_TOP_CORNER == type)
                || (LEFT_BOTTOM_CORNER == type)
                || (RIGHT_BOTTOM_CORNER == type)
                || (RIGHT_TOP_CORNER == type);
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

    private double getCenter(double x, double y) {
        return isVertical() ? x : y;
    }

    private int getCenter(int x, int y) {
        return isVertical() ? x : y;
    }

    private static int getTileXY(double tileXY) {
        return (int) (tileXY / tile_size);
    }

    void debugOffsets() {
        int[] wp = getNextWayPointXYFromIndex(wayPointIndex);
        log(direction + "[" + cx + "," + cy + "]"
                + " wp=" + world.getTilesXY()[self.getNextWaypointX()][self.getNextWaypointY()]
                + " [" + self.getNextWaypointX()
                + "," + self.getNextWaypointY()
                + "] next wp=" + world.getTilesXY()[wp[0]][wp[1]] + "[" + wp[0] + "," + wp[1] + "]"
                + " self=" + round(getCenter(self.getX(), self.getY()))
                + " center=" + centerXY
                + " axis=" + lastCenterXY
                + " offset=" + round(getOffsetCenter())
                + " angle=" + getOffsetAngle());
    }

    void debugTurn(NextTurn turn) {
        log("Turn  " + direction + " -> " + turn.direction1 + ", " + " --- " + turn.tile
                + ", dist=" + turn.distanceTo() + ", speed=" + speed);
    }

    /*
        public static void main(String[] args) {
            MyStrategy ms = new MyStrategy();

            int[][] waypoint = new int[][] {
                    {13, 15},
                    {1, 15},
                    {0, 0},
                    {2, 0},
                    {2, 14},
                    {13, 13}
            };
            TileType[][] tiles = new TileType[][] {
                    {LEFT_TOP_CORNER, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T,
                            RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, RIGHT_HEADED_T, LEFT_BOTTOM_CORNER},
                    {HORIZONTAL, RIGHT_TOP_CORNER, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T,
                            LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, LEFT_HEADED_T, TOP_HEADED_T},
                    {RIGHT_TOP_CORNER, RIGHT_HEADED_T, RIGHT_HEADED_T, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL,
                            RIGHT_HEADED_T, RIGHT_HEADED_T, VERTICAL, LEFT_BOTTOM_CORNER, HORIZONTAL},
                    {EMPTY, RIGHT_TOP_CORNER, LEFT_HEADED_T, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, VERTICAL, LEFT_HEADED_T,
                            RIGHT_BOTTOM_CORNER, LEFT_TOP_CORNER, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, RIGHT_BOTTOM_CORNER,
                            HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, LEFT_TOP_CORNER, LEFT_HEADED_T, LEFT_BOTTOM_CORNER,
                            HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, HORIZONTAL, EMPTY, HORIZONTAL, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, RIGHT_TOP_CORNER, RIGHT_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, TOP_HEADED_T, HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, LEFT_TOP_CORNER, LEFT_HEADED_T, RIGHT_BOTTOM_CORNER,
                            HORIZONTAL},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, BOTTOM_HEADED_T, VERTICAL, RIGHT_HEADED_T,
                            RIGHT_BOTTOM_CORNER},
                    {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, RIGHT_TOP_CORNER, VERTICAL, RIGHT_BOTTOM_CORNER, EMPTY},
            };

            ms.world = new World(0, 0, 0, tiles.length, tiles[0].length, new Player[0], new Car[0], new Projectile[0], new Bonus[0], new OilSlick[0], "", tiles,
                    waypoint, LEFT);
            ms.self = new Car(1, 1000, getTileCenter(9), getTileCenter(12), 0, 0, ANGLE_90, 0, 200, 400, 1, 0, true, CarType.BUGGY, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                    0, 13, 13, false);
            ms.init();
            long t = System.currentTimeMillis();
            WayResolver w = ms.getDirectionForClosestWayToTheNextWaypoint(9, 12, DOWN, 5, 1000);
            System.out.println(w);
            System.out.println(t - System.currentTimeMillis());
        }
    */
    static void log(Object str) {
        //
        System.out.println(str);
    }

}
