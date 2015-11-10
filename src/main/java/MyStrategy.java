import model.*;

import static java.lang.Math.*;

public final class MyStrategy implements Strategy {

    private static final double ANGLE_90 = 1.5708;
    private static final double ANGLE_45 = 1.5708 / 2;

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
        return (deltaSpeed() < 0.0001 && deltaMove() < 0.0001) && tickTr > 20;
    }

    void checkTurn() {
        TileType tt = getNext();

        if (tt != lastTile) {
            System.out.println("-" + lastTile + " -> " + tt);
            enableCenter = true;
            if (tt == TileType.LEFT_TOP_CORNER) {
                if (lastTile == TileType.VERTICAL && self.getSpeedY() < 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() < 0) {
                    delegate = new TurnLeft();
                }
            }

            if (tt == TileType.RIGHT_TOP_CORNER) {
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() > 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.VERTICAL && self.getSpeedY() < 0) {
                    delegate = new TurnLeft();
                }
            }

            if (tt == TileType.RIGHT_BOTTOM_CORNER) {
                if (lastTile == TileType.VERTICAL && self.getSpeedY() > 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() > 0) {
                    delegate = new TurnLeft();
                }
            }

            if (tt == TileType.LEFT_BOTTOM_CORNER) {
                if (lastTile == TileType.HORIZONTAL && self.getSpeedX() < 0) {
                    delegate = new TurnRight();
                }
                if (lastTile == TileType.VERTICAL && self.getSpeedY() > 0) {
                    delegate = new TurnLeft();
                }
            }
            lastTile = tt;
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

    void checkCenter() {
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
            move.setEnginePower(1);
            checkTurn();
            checkCenter();
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
        Turn() {
            tickTr = 0;
            turnAngleBegin = self.getAngle();
            System.out.println("New turn " + this.getClass());
        }

        abstract double get();

        @Override
        public void move() {
            move.setEnginePower(1.0);

            double d = self.getAngle() - turnAngleBegin;

            if (abs(d) >= ANGLE_90) {
                System.out.println("Angle id " + (self.getAngle() - turnAngleBegin) + " stop turn");
                delegate = new MoveForward();
            } else if (abs(d) >= ANGLE_45) {
                move.setWheelTurn(get() * d / ANGLE_90);
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

}
