import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static model.Direction.DOWN;
import static model.Direction.LEFT;
import static model.Direction.RIGHT;
import static model.Direction.UP;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import model.Bonus;
import model.BonusType;
import model.Car;
import model.CarType;
import model.CircularUnit;
import model.Direction;
import model.Game;
import model.Move;
import model.RectangularUnit;
import model.TileType;
import model.Unit;
import model.World;

public final class MyStrategy implements Strategy {

    static double mass_factor = 1.00 / 1250.00;

    static double inside_turn_factor = 0.0;//0..1
    static double outside_turn_factor = 1;//0..1
    static double mobility_factor = 0.8;
    static double stability_factor = 7;
    static double double_turn_u_speed = 7;
    static double turn_end_angle = 95;
    static double double_turn_z_end_angle = 45;
    static double double_turn_u_end_angle = 120;

    static double turn_max_speed = 18;
    static double turn_max_angular_speed = 0.033;
    static double turn_enter_offset_tiles = 0;
    static double drift_speed = 26;
    static boolean enable_nitro = true;
    static boolean enable_turn_enter_offset = true;
    static boolean enable_bonus_lookup = true;
    static boolean enable_car_avoidance = true;//false;//
    static boolean enable_corner_avoidance = true;
    static boolean enable_turn_enter_delay = false;
    static int stale_tick_count = 70;
    static double stale_speed_value = 0.1;
    static double stale_move_value = 1;
    static double max_speed = 1000;
    static double margins = 80;
    static double fromCenterToBorder = 320;
    static double fromCenterToBorderWithWidth = 260;

    private static final double[] OFFSET = new double[] {1, -1, -1, 1};
    private static final double[][] DIRECTION_XY = new double[][] {
            //      X   Y
            /* L */{-1, 0},
            /* R */{1, 0},
            /* U */{0, -1},
            /* D */{0, 1}
    };

    private static final double[][] TURN_DIRECTION = new double[][] {
            //      L   R  U  D
            /* L */{0, 0, 1, -1},
            /* R */{0, 0, -1, 1},
            /* U */{-1, 1, 0, 0},
            /* D */{1, -1, 0, 0}
    };

    private static final double[][] OFFS_ANGLE_SIGN = new double[][] {
            //         LEFT RIGHT UP DOWN
            /*LEFT  */{0, 0, -1, 1},
            /*RIGHT */{0, 0, 1, -1},
            /*UP    */{1, -1, 0, 0},
            /*DOWN  */{-1, 1, 0, 0}
    };

    private static final double[][] TURN_ANGLE_OUT = new double[][] {
            //        LEFT RIGHT UP DOWN
            /*LEFT  */{0, 0, 1, -1},
            /*RIGHT */{0, 0, -1, 1},
            /*UP    */{1, -1, 0, 0},
            /*DOWN  */{-1, 1, 0, 0}
    };
    private static final double[][] TURN_OFFSET_OUT = new double[][] {
            //         LEFT RIGHT UP DOWN
            /*LEFT  */{0, 0, 1, -1},
            /*RIGHT */{0, 0, 1, -1},
            /*UP    */{1, -1, 0, 0},
            /*DOWN  */{1, -1, 0, 0}
    };

    private static final double[][][] TURN_INSIDE_CORNER_XY = new double[][][] {
            //           LEFT    RIGHT    UP      DOWN
            /*LEFT  */{ {0, 0}, {0, 0}, {1, -1}, {1, 1}},
            /*RIGHT */{ {0, 0}, {0, 0}, {-1, -1}, {-1, 1}},
            /*UP    */{ {-1, 1}, {1, 1}, {0, 0}, {0, 0}},
            /*DOWN  */{ {-1, -1}, {1, -1}, {0, 0}, {0, 0}}
    };

    private static final double[][][] TURN_OUTSIDE_CORNER_XY = new double[][][] {
            //           LEFT   RIGHT   UP       DOWN
            /*LEFT  */{ {0, 0}, {0, 0}, {-1, -1.5}, {-1, 1.5}},
            /*RIGHT */{ {0, 0}, {0, 0}, {1, -1.5}, {1, 1.5}},
            /*UP    */{ {-1.5, -1}, {1.5, -1}, {0, 0}, {0, 0}},
            /*DOWN  */{ {-1.5, 1}, {1.5, 1}, {0, 0}, {0, 0}}
    };

    private static final double ANGLE_360 = 2 * PI;
    private static final double ANGLE_180 = PI;
    private static final double ANGLE_120 = 2 * PI / 3.0;
    private static final double ANGLE_90 = PI / 2.0;
    private static final double ANGLE_30 = PI / 6.0;
    private static final double ANGLE_15 = PI / 12.0;
    private static final double ANGLE_1 = PI / 180.0;

    private static double tile_size = 800;
    double A = 0.0;
    double B = 0.0;
    double C = 1.36;
    double D = 0.9;
    double E = 1.2;

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
    private double maxSpeed = max_speed;
    private double turnMaxSpeed = turn_max_speed;
    private double fireMaxDistance = 10000;
    private double oilMaxDistance = 10000;
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
        final double cornerX2;
        final double cornerY2;

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
            double[] c = getTurnTileInsideCornerCoord(this.x, this.y, this.direction0, this.direction1, margins + self.getHeight() / 2);
            this.cornerX = c[0];
            this.cornerY = c[1];
            c = getTurnTileOutsideCornerCoord(this.x, this.y, this.direction0, this.direction1);
            this.cornerX2 = c[0];
            this.cornerY2 = c[1];
        }

        double distanceTo() {
            return distance;
        }

        @Override
        public String toString() {
            return tile + " [" + tileX + "," + tileY + "] " + direction0 + " -> " + direction1;
        }
    }

    private NextTurn currentTurn;
    private NextTurn doubleTurn;
    private int doubleTurnUFinished;
    private double moveBackBeginX;
    private double moveBackBeginY;
    private double speed;
    private double accel;
    private boolean brakeOn;

    private int cx;
    private int cy;
    private TileType ctile;

    private double centerXY;
    private double currentCenterAxisXY;
    private Direction direction;
    private Bonus bonus;
    private Unit unitToAvoid;

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

        tile_size = (int) game.getTrackTileSize();

        staleMoveDelay = (int) (1.0 / game.getCarWheelTurnChangePerTick());
        margins = game.getTrackTileMargin();
        fromCenterToBorder = tile_size / 2 - margins;
        fromCenterToBorderWithWidth = fromCenterToBorder - self.getHeight() / 2;
        fireMaxDistance = tile_size * 3;
        oilMaxDistance = tile_size * 12;
        turn_max_speed = turn_max_speed;/// (mass_factor * self.getMass());
        drift_speed = drift_speed / (mass_factor * self.getMass());
    }

    int startX = -1;
    int startY = -1;

    @Override
    public void move(Car self, World world, Game game, Move move) {

        this.self = self;
        this.world = world;
        this.game = game;
        this.move = move;

        if (null == direction) {
            init();
        }
        turnEnterOffset = tile_size * turn_enter_offset_tiles;

        cx = getTileXY(self.getX());
        cy = getTileXY(self.getY());
        ctile = world.getTilesXY()[cx][cy];

        if (startX == cx && startY == cy) {
            cleared = new boolean[world.getWidth()][world.getHeight()];
        } else {
            cleared[cx][cy] = true;
        }

        currentCenterAxisXY = getTileCenter(getCenter(cx, cy));
        centerXY = currentCenterAxisXY;

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
        debugOffsets();
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
            backwardOn = false;
            move.setBrake(false);
            if (delegate instanceof MoveBackward) {
                tickTr = 0;
                delegate = ((MoveBackward) delegate).lastDelegate;
            } else {
                delegate = new MoveBackward(delegate);
            }
        }
    }

    //    double getTurnDelay2(double V, double a0, double a1) {
    //
    //        return min(1.5 * tile_size, 1 / game.getCarAngularSpeedFactor() / V * (sin(a0 + a1) - sin(a0)));
    //    }

    double getTurnDelay() {

        double start = max(tile_size / 2, A * speed / turnMaxSpeed + C * tile_size);
        if (speed > drift_speed) {
            start = speed / drift_speed * turnEnterOffset;
        } else if (null != doubleTurn) {
            start = doubleTurn.direction1 != direction ? turnEnterOffset * D : turnEnterOffset * E;
        }

        double offset = (getAxis(self.getX(), self.getY()) - currentCenterAxisXY)
                * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];

        if (offset < 0)
            start = tile_size / 2 + start * abs((fromCenterToBorderWithWidth + offset) / fromCenterToBorderWithWidth);

        return start;
    }

    NextTurn doubleTurnCheck(NextTurn turn) {
        int dX = NEXT_TURN_DELTAS[turn.direction1.ordinal()][0];
        int dY = NEXT_TURN_DELTAS[turn.direction1.ordinal()][1];
        int maxX = world.getWidth();
        int maxY = world.getHeight();
        for (int x = turn.tileX + dX, y = turn.tileY + dY; x < maxX && y < maxY && x > -1 && y > -1;) {
            final TileType tile = world.getTilesXY()[x][y];
            Direction[] directions = DIRECTION_MATRIX[turn.direction1.ordinal()][tile.ordinal()];
            if (null != directions && directions.length == 1 && directions[0] != turn.direction1) {
                return new NextTurn(x, y, turn.direction1, directions[0]);
            }
            break;
        }
        return null;
    }

    PathFinder pathFinder;

    void checkTurn() {

        if (turning)
            return;

        debugOffsets();

        if (null == pathFinder) {
            pathFinder = getDirectionForClosestWayToTheNextWaypoint(cx, cy, direction, wayPointIndex, Integer.MAX_VALUE);
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
                unitToAvoid = null;
                //
                if (path.isDouble()) {
                    doubleTurn = new NextTurn(pathFinder.child.x, pathFinder.child.y, path.direction, path.child.direction);
                }
                if (null == doubleTurn) {
                    doubleTurn = doubleTurnCheck(currentTurn);
                }
                pathFinder = pathFinder.child;

                log("Next turn detected: " + currentTurn + " double : " + doubleTurn);
            }
        }
        if (null != currentTurn) {
            double dist = currentTurn.distanceTo();

            double start = getTurnDelay();

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
        double delta1 = getStability() * getOffsetAngle() / ANGLE_180;

        //            log("Dir " + direction + " need=" + need + ", real=" + real + " OF " + DIR[direction.ordinal()]);

        double delta2 = getMobility() * getOffsetCenter() / fromCenterToBorder;

        if (abs(getOffsetCenter()) > fromCenterToBorder || abs(getOffsetAngle()) > ANGLE_90) {
            turnMaxSpeed = turn_max_speed / 3;
            maxSpeed = turn_max_speed / 2;
        } else {
            turnMaxSpeed = turn_max_speed;
            maxSpeed = max_speed;
        }

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

    void avoidCollisions() {

        double aCorner = 0;
        double d1 = 0;
        double angle = ANGLE_15;
        double x;
        double y;

        if (null != currentTurn && enable_corner_avoidance && currentTurn.turning) {
            if (cx != currentTurn.tileX || cy != currentTurn.tileY) {
                x = currentTurn.cornerX;
                y = currentTurn.cornerY;
                double a = self.getAngleTo(x, y);
                d1 = self.getDistanceTo(x, y);
                if (d1 < tile_size * 1.5) {
                    if (-a * currentTurn.turnDirection > 0) {
                        log("Avoid inside corner x=" + x + ", y=" + y + ", a=" + aCorner + ", d=" + d1
                                + " self.x=" + self.getX()
                                + " self.y=" + self.getY()
                                + " correction=" + aCorner);
                        //                        a2 = (a2);
                        aCorner = -0.5 * currentTurn.turnDirection;
                    }
                }
            } else {
                x = currentTurn.cornerX2;
                y = currentTurn.cornerY2;
                double a = currentTurn.turnDirection * self.getAngleTo(x, y);
                d1 = self.getDistanceTo(x, y);
                if (d1 < tile_size / 1.5) {
                    if (a > -ANGLE_1 * 15) {
                        log("Avoid outside corner x=" + x + ", y=" + y + ", a=" + aCorner + ", d=" + d1
                                + " self.x=" + self.getX()
                                + " self.y=" + self.getY()
                                + " correction=" + a);
                        aCorner = 100 * currentTurn.turnDirection * abs(a);
                    }
                }
            }
        }

        Car carToAvoid = null;
        double d2 = 1;
        if (enable_car_avoidance) {
            double minD = tile_size;
            for (Car car : world.getCars()) {
                if (self.getId() != car.getId()) {
                    d2 = self.getDistanceTo(car);
                    double a = self.getAngleTo(car);
                    if (d2 < minD) {
                        if (abs(a) < ANGLE_30 && abs(tan(a) * d2) < car.getHeight() / 2) {
                            carToAvoid = car;
                            minD = d2;
                        }
                    }
                }
            }
        }
        if (null != carToAvoid) {
            double a2 = self.getAngleTo(carToAvoid);
            aCorner = aCorner * d1 / d2;
            a2 = a2 * (angle - abs(a2)) / angle * d2 / d1;

            log("Avoid car a1=" + aCorner + ", a2=" + a2 + ", d1=" + d1 + ", d2=" + d2);
            if (signum(a2) == signum(aCorner)) {
                aCorner = maxByAbs(aCorner, a2);
            } else {
                aCorner = aCorner + a2;
            }

        }
        if (aCorner != 0) {
            move.setWheelTurn(aCorner);
        }
    }

    void defineCenter() {
        if (!backwardOn) {

            maxSpeed = max_speed;
            turnMaxSpeed = turn_max_speed;
            //Прижаться к краю
            //centerX += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();
            //centerY += TURN_OFFSET_OUT[direction.ordinal()][nextTurnDirection.ordinal()] * self.getWidth();

            if (enable_turn_enter_offset && null != currentTurn) {
                double m = currentTurn.distanceTo() < tile_size && speed < turnMaxSpeed ? -insideTurnFactor : outside_turn_factor;

                if (speed > drift_speed) {
                    m = 0.9;
                }

                if (null != doubleTurn) {
                    if (doubleTurn.direction1 != direction) {
                        m = 1;
                    } else {
                        m = 0;
                    }
                }

                centerXY = currentCenterAxisXY + m * fromCenterToBorderWithWidth * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];
            }

            pickBonus();
            avoidCar();

            //Check limits
            double deltaCXY = currentCenterAxisXY - centerXY;
            if (abs(deltaCXY) > fromCenterToBorderWithWidth) {
                centerXY = currentCenterAxisXY + signum(centerXY - currentCenterAxisXY) * fromCenterToBorderWithWidth;
            }

        }
    }

    double getDistanceTo(Unit unit) {
        return getDistanceTo(unit.getX(), unit.getY());
    }

    double getDistanceTo(double x, double y) {
        return DIRECTION_XY[direction.ordinal()][0] * (x - self.getX()) + DIRECTION_XY[direction.ordinal()][1] * (y - self.getY());
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

                    double bonusOffCenter = getDistFromCenterAxis(b);

                    double distToBonus = getDistanceTo(b);

                    if (distToBonus > tile_size
                            && abs(getAxis(self) - getAxis(b)) < fromCenterToBorder
                            && abs(bonusOffCenter) < fromCenterToBorder
                            && (null == currentTurn || distToBonus < currentTurn.distance - tile_size)) {
                        if (distToBonus < closestXY && (null == bonus || bonus.getType() != BonusType.PURE_SCORE || bonus.getType() != BonusType.REPAIR_KIT)) {
                            int x = getTileXY(b.getX());
                            int y = getTileXY(b.getY());
                            boolean insideTurn = null != currentTurn && currentTurn.tileX == x && currentTurn.tileY == y;
                            if (!insideTurn) {
                                closestXY = distToBonus;
                                bonus = b;
                            }
                        }
                    }
                }

            } else {
                if (getDistanceTo(bonus) <= tile_size / 2) {
                    bonus = null;
                } else {
                    centerXY = getAxis(bonus.getX(), bonus.getY()) - bonus.getHeight() / 2 * signum(getDistFromCenterAxis(bonus));
                }
            }
        }
    }

    void avoidCar() {
        if (enable_car_avoidance && !turning) {
            if (null == unitToAvoid) {
                double closestXY = Double.MAX_VALUE;
                Unit[] units = new Unit[world.getCars().length + world.getOilSlicks().length];
                System.arraycopy(world.getCars(), 0, units, 0, world.getCars().length);
                System.arraycopy(world.getOilSlicks(), 0, units, world.getCars().length, world.getOilSlicks().length);
                for (Unit u : units) {
                    if (u.getId() == self.getId()) {
                        continue;
                    }
                    double offsetToUnit = currentCenterAxisXY - getAxis(u.getX(), u.getY());
                    double distToUnit = getDistanceTo(u);
                    if (distToUnit > 0 && abs(offsetToUnit) < fromCenterToBorder && (null == currentTurn || distToUnit < currentTurn.distance)
                            && getSpeedAlongAxis(u.getSpeedX(), u.getSpeedX()) <= getSpeedAlongAxis(self.getSpeedX(), self.getSpeedX())) {
                        if (distToUnit < closestXY) {
                            double offsetXY = getAxis(self.getX(), self.getY()) - getAxis(u.getX(), u.getY());
                            if (abs(offsetXY) < self.getWidth()) {
                                int x = getTileXY(u.getX());
                                int y = getTileXY(u.getY());
                                boolean turn = null != currentTurn && currentTurn.tileX == x && currentTurn.tileY == y;
                                if (!turn) {
                                    closestXY = distToUnit;
                                    unitToAvoid = u;
                                }
                            }
                        }
                    }
                }
            } else {
                if (getDistanceTo(unitToAvoid) <= 0) {
                    unitToAvoid = null;
                } else {
                    double carCenter = getAxis(unitToAvoid.getX(), unitToAvoid.getY());
                    double size = unitToAvoid instanceof CircularUnit
                            ? ((CircularUnit) unitToAvoid).getRadius()
                            : ((RectangularUnit) unitToAvoid).getHeight();
                    if (currentCenterAxisXY + fromCenterToBorderWithWidth < carCenter + size) {
                        centerXY = carCenter - size;
                    }
                    if (currentCenterAxisXY - fromCenterToBorderWithWidth > carCenter - size) {
                        centerXY = carCenter + size;
                    }
                }
            }
        }
    }

    double getOffsetCenter() {
        double offset = getAxis(self.getX(), self.getY()) - centerXY;
        return offset * OFFSET[direction.ordinal()];
    }

    double getOffsetRealCenter() {
        double offset = getAxis(self.getX(), self.getY()) - currentCenterAxisXY;
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
            checkStraight();
            avoidCollisions();
            checkBrake();
            checkNitro();
            checkDistance();
            checkCanThrowProjectile();
            checkStale();
        }
    }

    double getMobility() {
        return turn_max_speed / speed * mobility_factor;
    }

    double getStability() {
        return speed / turn_max_speed * stability_factor;
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
        int phase = 0;
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
            offsAngle = getOffsetAngle() > 0 ? 1 : -1;
            backwardOn = true;
        }

        @Override
        public void move() {
            //            if (countDownMove1 > 0) {
            //                countDownMove1--;
            //                tickTr = 0;//Prevent stale check
            //                move.setBrake(true);
            //                move.setEnginePower(0.0);
            //            }
            checkStraight();
            if (0 == phase && self.getWheelTurn() != -offsAngle) {
                move.setWheelTurn(-offsAngle);
            }
            if (0 == phase && self.getWheelTurn() == -offsAngle) {
                phase++;
            }

            if (1 == phase && deltaBack() < tile_size / 4 && speed < 10) {
                move.setWheelTurn(-offsAngle);
                move.setEnginePower(-1.0);
            }
            if (1 == phase && deltaBack() > tile_size / 4 || speed > 10) {
                move.setWheelTurn(offsAngle);
                phase++;
            }

            if (2 == phase && speed > stale_speed_value) {
                move.setBrake(true);
                move.setEnginePower(0.0);
                move.setWheelTurn(offsAngle);
                //Prevent stale check
                tickTr = 0;
            }
            if (2 == phase && speed < stale_speed_value) {
                phase++;
            }

            if (3 == phase) {
                backwardOn = false;
                delegate = lastDelegate;
            }
            checkStale();
        }
    }

    boolean checkFire(double distance) {
        for (Car car : world.getCars()) {
            if (!car.isTeammate()) {
                if ((self.getType() == CarType.BUGGY && canFire(car))
                        || (self.getType() == CarType.JEEP && canFireJeep(car))) {
                    return true;
                }
            } else if (car.getId() != self.getId()) {
                ally = car;
            }
        }
        return false;
    }

    void checkCanThrowProjectile() {
        if (self.getProjectileCount() > 0) {
            if (checkFire(fireMaxDistance)) {
                move.setThrowProjectile(true);
            }
        }
        if (self.getOilCanisterCount() > 0) {
            if (null != currentTurn && currentTurn.distance < 2 * tile_size || (ctile != TileType.HORIZONTAL && ctile != TileType.VERTICAL)) {
                if (checkCloseAngle(ANGLE_180, ANGLE_90, oilMaxDistance)) {
                    move.setSpillOil(true);
                }
            }
        }
    }

    boolean isCanUseNitro() {

        int ts = (tickCount < 10) ? 0 : 4;

        boolean b = true;// abs(abs(getRightAngle()) - abs(self.getAngle())) < ANGLE_1 * 5 && abs(getOffsetRealCenter()) < fromCenterToBorderWithWidth;

        if (null != currentTurn) {
            b = b && currentTurn.distance > tile_size * ts;
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
        brakeOn = speed > maxSpeed;
        turnMaxSpeed = turn_max_speed;

        if (turning || (null != currentTurn && currentTurn.distanceTo() < 1.36 * tile_size)) {
            if (doubleTurnUFinished > 0) {
                turnMaxSpeed = double_turn_u_speed;
                log("Double turn U max speed " + turnMaxSpeed);
            } else if (null != doubleTurn && doubleTurn.direction1 != currentTurn.direction0) {
                turnMaxSpeed = double_turn_u_speed;
                log("Double turn U max speed " + turnMaxSpeed);
            } else {
                turnMaxSpeed = turnMaxSpeed + 1 * self.getDurability();

                double offset = (getAxis(self.getX(), self.getY()) - currentCenterAxisXY)
                        * TURN_OFFSET_OUT[direction.ordinal()][currentTurn.direction1.ordinal()];

                if (offset < 0)
                    turnMaxSpeed = turnMaxSpeed / 2 + turnMaxSpeed / 2 * abs((fromCenterToBorderWithWidth + offset) / fromCenterToBorderWithWidth);
            }
            brakeOn = speed > turnMaxSpeed;

            log("Brake on speed " + speed + " max speed " + turnMaxSpeed);
        }
        if (brakeOn) {
            //            log("Bake ON! " + speed);
            log("Brake on! : " + currentTurn);
            move.setBrake(true);
        }
    }

    boolean checkCloseAngle(double angle, double spread, double distance) {
        for (Car car : world.getCars()) {
            if (!car.isTeammate()) {
                double d = self.getDistanceTo(car);
                double a = self.getAngleTo(car);
                a = a - signum(a) * angle;

                if (d < distance && abs(a) < spread) {
                    return true;
                }
            } else if (car.getId() != self.getId()) {
                ally = car;
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

        double endDeltaAngle = ANGLE_1 * turn_end_angle;
        double turnSpeedStart;
        double side;

        Turn() {
            bonus = null;
            unitToAvoid = null;
            currentTurn.turning = true;
            tickTr = 0;
            turnAngleBegin = getRightAngle();
            debugOffsets();
            side = currentTurn.turnDirection;
            turning = true;

            if (currentTurn.entered) {
                direction = currentTurn.direction1;
            }
            if (null != doubleTurn) {
                endDeltaAngle = doubleTurn.direction1 == direction
                        ? double_turn_z_end_angle * ANGLE_1
                        : double_turn_u_end_angle * ANGLE_1;
                if (doubleTurn.direction1 != currentTurn.direction0)
                    doubleTurnUFinished = 2;
            }
            turnSpeedStart = turnMaxSpeed;
            log("New turn " + this.getClass() + " from angle: " + turnAngleBegin + " to angle: " + endDeltaAngle
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

            if (null == doubleTurn || doubleTurn.direction1 == direction)
                turnMaxSpeed = turnSpeedStart
                        + 0.5 * self.getWidth() / self.getDistanceTo(currentTurn.cornerX, currentTurn.cornerY) * turnSpeedStart * delta / endDeltaAngle;

            boolean inside = cx == currentTurn.tileX && cy == currentTurn.tileY;

            // Установить новое направление
            if (!currentTurn.entered && inside) {
                currentTurn.entered = true;
                direction = currentTurn.direction1;
                pathFinder = getDirectionForClosestWayToTheNextWaypoint(currentTurn.tileX, currentTurn.tileY, direction, wayPointIndex, Integer.MAX_VALUE);
            }

            // exit by off turn tile
            boolean exitAlways = currentTurn.entered && !inside;

            if ((currentTurn.entered && delta >= endDeltaAngle) || exitAlways) {
                log("Turn finished: Angle is " + self.getAngle() + ", delta is " + delta);
                debugOffsets();
                log("--------------------------------------------------------------------------------------------------------------------");
                currentCenterAxisXY = getAxis(currentTurn.x, currentTurn.y);
                currentTurn = null;
                doubleTurn = null;
                doubleTurnUFinished--;
                delegate = null;
                turning = false;
                insideTurnFactor = inside_turn_factor;
                return;
            } else {
                //                double w = (turn_max_angular_speed - abs(self.getAngularSpeed())) / turn_max_angular_speed;
                //                (speed > drift_speed || inside ? 1 : 0.5)
                double wht = endDeltaAngle > ANGLE_90 ? 1 : (endDeltaAngle - delta) / endDeltaAngle;
                //                double wht = endDeltaAngle > ANGLE_90 ? 1 : cos(ANGLE_90 * delta / endDeltaAngle);
                move.setWheelTurn(get() * wht);
                checkBrake();
            }
            //            if (speed < drift_speed)
            avoidCollisions();
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

    private static boolean disableLogging;

    static void logIdent(int ident, String str) {
        log(String.format("%1$" + 3 * ident + "s", " ") + str);
    }

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
        int id;
        int gen = 1;
        public int waypoint = 0;
        public int offpoint = 0;
        public boolean exited;
        public boolean invalid;
        public boolean cycle;

        // For debug
        PathFinder(Direction dir, PathFinder par, int x, int y, int[] wp) {
            this.id = ++branchIndex;
            this.direction = dir;
            this.parent = par;
            if (par != null) {
                if (dir != par.direction)
                    par.turnCount++;
                turnCount = par.turnCount;
                gen = par.gen + 1;
            }
            this.x = x;
            this.y = y;
            this.wpx = wp[0];
            this.wpy = wp[1];
            logIdent(gen, "New " + this);
        }

        int getTurnCount() {
            return turnCount;
        }

        void selectChild(PathFinder child) {
            this.turnCount = child.turnCount;
            if (child.cycle) {
                this.cycle = true;
            }
            if (child.exited) {
                this.exited = true;
            }
            if (child.waypoint > 0) {
                this.waypoint = child.waypoint;
            }
            if (child.offpoint > 0) {
                this.offpoint = child.offpoint;
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

        public boolean isDouble() {
            return child != null && child.direction != direction
                    && (((child.x - x) == 0 && abs(child.y - y) == 1) || ((child.y - y) == 0 && abs(child.x - x) == 1));
        }

        @Override
        public int compareTo(PathFinder o) {
            if (waypoint != o.waypoint) {
                return o.waypoint - waypoint;
            }
            if (offpoint != o.offpoint) {
                return offpoint - o.offpoint;
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
            return "Way <" + (null != parent ? parent.id + "->" : "") + id + ", "
                    + (null != parent ? parent.direction + "->" : "")
                    + direction + " ["
                    + x + "," + y + "] count=" + turnCount
                    + ", exited=" + exited
                    + ", cycled=" + cycle
                    + ", invalid=" + invalid
                    + ", waypoint=" + waypoint
                    + ", double=" + isDouble()
                    + ">";
        }

    }

    Direction[] sortDirections(final Direction[] array, final int x, final int y, final int[] waypoint) {
        final int dx = abs(x - waypoint[0]);
        final int dy = abs(y - waypoint[1]);
        Comparator<Direction> c = new Comparator<Direction>() {
            @Override
            public int compare(Direction o1, Direction o2) {
                if (LEFT == o1 && x > waypoint[0] && dy < dx) {
                    return -1;
                }
                if (RIGHT == o1 && x < waypoint[0] && dy < dx) {
                    return -1;
                }
                if (UP == o1 && y > waypoint[1] && dy > dx) {
                    return -1;
                }
                if (DOWN == o1 && y < waypoint[1] && dy > dx) {
                    return -1;
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

        logIdent(pathFinder.gen, "Enter: " + pathFinder.direction + " [" + fromX + "," + fromY + "], waypointIndex="
                + waypointIndex + " ["
                + world.getWaypoints()[waypointIndex][0]
                + "," + world.getWaypoints()[waypointIndex][1] + "]");

        //Limit lookup
        if (pathFinder.getTurnCount() > 10) {
            logIdent(pathFinder.gen, "Exit: " + pathFinder + " limit reached for " + pathFinder);
            return;
        }

        if (cleared[fromX][fromY]) {
            pathFinder.cycle = true;
        }

        int[] nextWaypoints = world.getWaypoints()[waypointIndex];
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
                pathFinder.waypoint++;
                waypointIndex = getNextWayPoint(waypointIndex);
                nextWaypoints = world.getWaypoints()[waypointIndex];
                if (pathFinder.turnCount > 0) {
                    break;
                }
            } else if (untouchedWaypoints[x][y]) {
                pathFinder.offpoint++;
                // Exclude path that leads to waypoints out off order
                //                pathFinder.invalid = true;
                //                log("Exit: " + pathFinder + " Invalid waypoint founded, exit this way");
                //                return;
            }

            if (directions.length == 1 && fromDir == directions[0]) {
                continue;
            }

            if (!visitedTiles[x][y]) {
                visitedTiles[x][y] = true;

                if (null != directions) {
                    logIdent(pathFinder.gen, tile + " [" + x + "," + y + "] -> " + Arrays.asList(directions));
                    if (directions.length == 1) {
                        PathFinder way = new PathFinder(directions[0], pathFinder, x, y, nextWaypoints);
                        resolveClosestPathToWaypoint(x, y, waypointIndex, way, visitedTiles);
                        pathFinder.selectChild(way);
                        logIdent(pathFinder.gen, "Exit: " + pathFinder + " with single child " + way);
                        return;
                    } else {
                        List<PathFinder> ways = new ArrayList<>(directions.length);
                        //                        Direction[] directions0 = sortDirections(directions, x, y, nextWaypoints);
                        for (Direction d : directions) {
                            PathFinder finder = new PathFinder(d, pathFinder, x, y, nextWaypoints);
                            resolveClosestPathToWaypoint(x, y, waypointIndex, finder, visitedTiles);
                            ways.add(finder);
                        }
                        PathFinder way = pathFinder.selectChild(ways);
                        logIdent(pathFinder.gen, "Exit: " + pathFinder + " with selected child " + way);
                        return;
                    }

                }
            } else {
                logIdent(pathFinder.gen, "Skip: [" + x + "," + y + "] is visited");
            }
        }
        if (pathFinder.waypoint > 0) {
            logIdent(pathFinder.gen, "Exit: " + pathFinder + " Waypoint founded, exit this way");
        } else {
            pathFinder.exited = true;
            logIdent(pathFinder.gen, "Exit: " + pathFinder + " No suitable directions");
        }
    }

    PathFinder getDirectionForClosestWayToTheNextWaypoint(int fromX, int fromY, Direction dir, int waypointIndex, int limit) {

        disableLogging = true;
        try {
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

                if (x == nextWaypoints[0] && y == nextWaypoints[1]) {
                    waypointIndex = getNextWayPoint(waypointIndex);
                    nextWaypoints = world.getWaypoints()[waypointIndex];
                    log("Enter waypoint, increment. next is " + waypointIndex);
                }

                if (directions.length == 1 && dir == directions[0]) {
                    continue;
                }

                log("Directions [" + x + "," + y + "] " + Arrays.asList(directions));
                if (directions.length == 1) {
                    log("Exit: single way is " + directions[0]);
                    PathFinder finder = new PathFinder(directions[0], null, x, y, nextWaypoints);
                    resolveClosestPathToWaypoint(x, y, waypointIndex, finder, visitedTiles);
                    return finder;
                }
                List<PathFinder> ways = new ArrayList<>(directions.length);
                for (Direction d : directions) {
                    PathFinder finder = new PathFinder(d, null, x, y, nextWaypoints);
                    resolveClosestPathToWaypoint(x, y, waypointIndex, finder, visitedTiles);
                    ways.add(finder);
                }
                if (!ways.isEmpty()) {
                    Collections.sort(ways);
                    log("Exit: closest way is " + ways.get(0));
                    return ways.get(0);
                }
            }
            log("Exit: No suitable directions");
            return null;
        } finally {
            disableLogging = false;
        }
    }

    int getNextWayPoint(int waypointIndex) {
        waypointIndex++;
        if (waypointIndex == world.getWaypoints().length) {
            waypointIndex = 0;
        }
        return waypointIndex;
    }

    static int[][] NEXT_TURN_DELTAS = {
            //X, Y
            {-1, 0},//L  
            {1, 0},//R
            {0, -1},//U
            {0, 1},//D
    };

    TileType getTile(double x, double y) {
        return world.getTilesXY()[getTileXY(x)][getTileXY(y)];
    }

    double getDistFromCenterAxis(Unit unit) {
        double unitXY = getAxis(unit.getX(), unit.getY());
        return unitXY - currentCenterAxisXY;
    }

    double getDistFromAxis(Unit unit) {
        double unitXY = getAxis(unit.getX(), unit.getY());
        return unitXY - centerXY;
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

    private double getAxis(double x, double y) {
        return isVertical() ? x : y;
    }

    private double getAxis(Unit u) {
        return isVertical() ? u.getX() : u.getY();
    }

    private double getSpeedAlongAxis(double x, double y) {
        return DIRECTION_XY[direction.ordinal()][0] * x + DIRECTION_XY[direction.ordinal()][0] * y;
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
                + " self=" + round(getAxis(self.getX(), self.getY()))
                + " axis=" + currentCenterAxisXY
                + " axisoffset=" + round(getOffsetRealCenter())
                + " offset=" + round(getOffsetCenter())
                + " center=" + centerXY
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
        if (!disableLogging)
            System.out.println(str);
    }

    static double[] getTurnTileInsideCornerCoord(double tx, double ty, Direction d1, Direction d2, double offset) {
        double[] off = TURN_INSIDE_CORNER_XY[d1.ordinal()][d2.ordinal()];
        double x = tx + off[0] * (tile_size / 2 - offset);
        double y = ty + off[1] * (tile_size / 2 - offset);
        return new double[] {x, y};
    }

    static double[] getTurnTileOutsideCornerCoord(double tx, double ty, Direction d1, Direction d2) {
        double[] off = TURN_OUTSIDE_CORNER_XY[d1.ordinal()][d2.ordinal()];
        double x = tx + off[0] * (tile_size / 2 - margins);
        double y = ty + off[1] * (tile_size / 2 - margins);
        return new double[] {x, y};
    }

    boolean canFire(Car car) {

        if (car.isTeammate()) {
            return false;
        }

        double vx00 = game.getWasherInitialSpeed() * cos(self.getAngle());
        double vy00 = game.getWasherInitialSpeed() * sin(self.getAngle());
        double vx01 = game.getWasherInitialSpeed() * cos(self.getAngle() + game.getSideWasherAngle());
        double vy01 = game.getWasherInitialSpeed() * sin(self.getAngle() + game.getSideWasherAngle());
        double vx02 = game.getWasherInitialSpeed() * cos(self.getAngle() - game.getSideWasherAngle());
        double vy02 = game.getWasherInitialSpeed() * sin(self.getAngle() - game.getSideWasherAngle());

        double vx1 = car.getSpeedX();
        double vy1 = car.getSpeedY();

        double x0 = self.getX();
        double y0 = self.getY();
        double x1 = car.getX();
        double y1 = car.getY();

        double cs = sqrt(pow(vx1, 2) + pow(vy1, 2));
        double d = cs == 0 ? car.getWidth() / 2 : car.getWidth() / 2 / cs;

        return abs(self.getAngleTo(car)) < ANGLE_1 * 45 && (abs((x1 - x0) / (vx00 - vx1) - (y1 - y0) / (vy00 - vy1)) < d
                || abs((x1 - x0) / (vx01 - vx1) - (y1 - y0) / (vy01 - vy1)) < d
                || abs((x1 - x0) / (vx02 - vx1) - (y1 - y0) / (vy02 - vy1)) < d);
    }

    boolean canFireJeep(Car car) {

        double vx0 = game.getWasherInitialSpeed() * cos(self.getAngle());
        double vy0 = game.getWasherInitialSpeed() * sin(self.getAngle());
        double vx01 = game.getWasherInitialSpeed() * cos(self.getAngle() + game.getSideWasherAngle());
        double vy01 = game.getWasherInitialSpeed() * sin(self.getAngle() + game.getSideWasherAngle());
        double vx02 = game.getWasherInitialSpeed() * cos(self.getAngle() - game.getSideWasherAngle());
        double vy02 = game.getWasherInitialSpeed() * sin(self.getAngle() - game.getSideWasherAngle());

        double vx1 = car.getSpeedX();
        double vy1 = car.getSpeedY();

        double x0 = self.getX();
        double y0 = self.getY();
        double x1 = car.getX();
        double y1 = car.getY();

        double d = car.getWidth() / 2 / sqrt(pow(vx1, 2) + pow(vy1, 2));

        return abs((x1 - x0) / (vx0 - vx1) - (y1 - y0) / (vy0 - vy1)) < d
                || abs((x1 - x0) / (vx01 - vx1) - (y1 - y0) / (vy01 - vy1)) < d
                || abs((x1 - x0) / (vx02 - vx1) - (y1 - y0) / (vy02 - vy1)) < d;
    }
}
