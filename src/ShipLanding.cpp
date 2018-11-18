/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

/*-Local defines--------------------------------------------------------------*/

static const bool UNITTEST = false;

/*
 * WARNING:
 * IF YOU CHANGE ANY OF THOSE, YOU BETTER MAKE SURE THAT THEY STILL FIT IN THE
 * GIVEN DATATYPE!
 */

static const uint16_t MAX_DISTANCE_TO_SHIP = 300;      // meter
static const uint8_t LOITER_DISTANCE_TO_SHIP = 100;    // meter
static const uint8_t LOITER_UPDATE = 10;               // second
static const uint8_t LOITER_ALTITUDE = 50;             // meter
static const double HEADING_WEIGHT = 0.1;

/*
 * The following are necessary because the heading doesn't work like angles in
 * a normal coordinate system (starting at the x-axis with 0 degrees going
 * counterclockwise). Instead, it starts at NORTH (y-axis), going clockwise.
 */

static const uint8_t NORTH = 0; //degrees
static const uint8_t EAST = 90; //degrees
static const uint8_t SOUTH = 180; //degrees
static const uint16_t WEST = 270; //degrees

/*-Public functions-----------------------------------------------------------*/

void ShipLanding::release()
{
    if (_instance != nullptr)
        delete _instance;
    _instance = nullptr;
}

QObject* ShipLanding::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine);
    Q_UNUSED(scriptEngine);

    if (_instance == nullptr)
        _instance = new ShipLanding();
    return _instance;;
}

/*-Public slots---------------------------------------------------------------*/

void ShipLanding::initDialog()
/** Message Box to init the landing. */
{
    qDebug() << "Init Dialog yes";
    emit confirmLanding();
}

void ShipLanding::cancelDialog()
/** Message Box to cancel the Landing. */
{
    qDebug() << "Cancel Dialog yes";
    emit confirmCancel();
}

/*-Private functions----------------------------------------------------------*/

ShipLanding::ShipLanding(QObject *parent) : QObject(parent), _vehicle(nullptr)
  /** Connects slots and signals, configures the timerLoiter. */
{
    // Connect our signals to corresponding functions.
    connect(this, &ShipLanding::confirmLanding, this, &ShipLanding::land);
    connect(this, &ShipLanding::confirmCancel,
            this, &ShipLanding::prepareToLoiter);

    // TODO: Connect GQCPositionManager to USB_GPS.
    connect(qgcApp()->toolbox()->qgcPositionManager(),
            &QGCPositionManager::positionInfoUpdated,
            this,
            &ShipLanding::update_posPlane);

    // TODO: Position plane.
    /*connect(qgcApp()->toolbox()->PositionManager(),
              &PlanePositionManager::positionInfoUpdated,
              this,
              &ShipLanding::update_posPlane);*/

    // Timers
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterShip);

    // Vehicle
    if(qgcApp()->toolbox()->multiVehicleManager()->activeVehicle())
        _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();

    // TODO: Connect prepare to land to end of mission
}

ShipLanding::~ShipLanding()
{
}

QGeoCoordinate ShipLanding::calcLoiterPos()
/** Calculate the position LOITER_DISTANCE_TO_SHIP away from ship, resting upon
    the heading. */
{
    if (UNITTEST)
    {
        ship.dir =225;
        ship.coord.setAltitude(500);
        ship.coord.setLatitude(47.4065160);
        ship.coord.setLongitude(8.5425730);
    }

    QGeoCoordinate pos;
    double longitude = 0, latitude = 0;

    longitude = sin(ship.dir * M_PI / 180) * LOITER_DISTANCE_TO_SHIP;
    latitude = cos(ship.dir * M_PI / 180) * LOITER_DISTANCE_TO_SHIP;

    pos.setLatitude(ship.coord.latitude() - (latitude/111300));
    pos.setLongitude(ship.coord.longitude() -
                  (longitude/(111300*cos(ship.coord.latitude()*(M_PI / 180)))));
    pos.setAltitude(LOITER_ALTITUDE);

    if (UNITTEST)
        qDebug() << pos.longitude() << "||" << pos.latitude() << "||" <<
             longitude << "||" << latitude << "||" <<ship.coord.distanceTo(pos);

    return pos;
}

void ShipLanding::start_timerLoiter()
/** Start the timer for loiter update. */
{
    timerLoiter->start(LOITER_UPDATE * 1000);
}

void ShipLanding::stop_timerLoiter()
/** Stop the timer for loiter update. */
{
    timerLoiter->stop();
}

/*-Private Slots--------------------------------------------------------------*/

void ShipLanding::prepareToLoiter()
/** Entry point for the landing procedure.
 * Starts the timerLoiter. Asks for the user's okay to init the landing. */
{
    /* TODO: Is interfacing this really needed? */
    start_timerLoiter();
}

void ShipLanding::loiterShip()
/** Build and send the loiter message to the plane. */
{
    if (ship.coord.distanceTo(plane.coord) > MAX_DISTANCE_TO_SHIP)
    {
        /** Send the plane 100m behind the ship.*/
        if(_vehicle)
        {
            _vehicle->guidedModeGotoLocation(calcLoiterPos());
        }
    }
}

void ShipLanding::land()
/** Called after user starts the landing. Stops the timerLoiter.
 * Build and send landing mission to plane. Observe the ship movement.
 * Provides the cancel option for the user. */
{
    /* TODO: Is interfacing this really needed? */
    stop_timerLoiter();                       // stop GPS from initiating Loiter

   /*
    * Something something landing
    * Ideas what could happen here:
    *  - mission + second mission in background
    *  - mission + check for missed boundaries via geofence + check for proper height with altitude check
    *  - mission + L1 library
    */

    // 1. WP: Loiter
    // 2. WP: Loiter down to goal height
    // 3. WP: Net
    // 4. WP: Behind ship so that we won't loiter
    // After 4. WP: Go back to WP2 and start again (some height)

    // Erweiterung: Beurteilung - Schaffe ich es noch? Ab Headingsänderung so und so lade neue Mission hoch
    // Bei Übertritt Geofence -> Schräg hochziehen

    // TODO: Point of no return reached -> User cannot stop this!
    // Left geofence -> return behind ship

    // TODO: Failsafe prepareToLoiter
}

void ShipLanding::update_posPlane()
/** Called when the plane moved. Update the saved location and heading. */
{
    // Transfer GPS data to struct
    // TODO: QGCPositionManager => __GPS
}

void ShipLanding::update_posShip()
/** Called when the ship moved. Update the saved location and heading. */
{
   /*
    * Safe away the old GPS data to calculate heading later.
    * @SBR: Please don't touch :)
    * TODO: Remove all comments containing "@" in final code.
    */
    QGeoCoordinate old_ship_pos = this->ship.coord;

    //@SBR: Start writing here.

    // Transfer GPs data to struct
    // TODO: QGCPositionManager => __GPS

    //@SBR: Stop writing here.

   /*
    * Calculate heading.
    * Idea: Start heading with 0 degrees.
    * Afterwards, calculate a heading from old and new GPS.
    * Add this heading to the new one, but weighed in with a factor.
    * Please note that this won't work at 180 degrees longitude, but we are at
    * mediteranian sea and this is supposed to be a workaround anyway, so I
    * won't bother building this to work at 180 degrees.
    */

    double alpha; //Alpha shows the angle in a normal coordinate system.
    double new_dir; //New direction.
    QGeoCoordinate new_ship_pos = this->ship.coord; //New ship position.

    if (new_ship_pos.longitude() == old_ship_pos.longitude())
    {
       /*
        * We cannot calculate alpha in this case, because we would divide by
        * zero.
        */
        if (new_ship_pos.latitude() == old_ship_pos.latitude())
            return; //Heading doesn't change if the positions are equal
        else if (new_ship_pos.latitude() > old_ship_pos.latitude())
            new_dir = NORTH;
        else
            new_dir = SOUTH;
    }
    else
    {
        alpha = atan((new_ship_pos.latitude() - old_ship_pos.latitude())/
                     (new_ship_pos.longitude() - old_ship_pos.longitude()));

       /*
        * First and second quadrant. If alpha turns out negative, it's second
        * quadrant. If alpha turns out positive, it's in the first one.
        */
        if (new_ship_pos.longitude() > old_ship_pos.longitude())
            new_dir = EAST - alpha;
        /*
         * Third and fourth quadrant. If alpha turns out negative, it's fourth
         * quadrant. If alpha turns out positive, it's in the third one.
         */
        else
            new_dir = WEST - alpha;
    }

   /*
    * If new heading is larger than old, add it. If smaller, substract it. And
    * if they're equal, the old heading isn't changed at all.
    */
    this->ship.dir = this->ship.dir + (HEADING_WEIGHT * (new_dir - this->ship.dir));
    if (new_dir > this->ship.dir)
        this->ship.dir += (HEADING_WEIGHT * new_dir);
    else if (new_dir < this->ship.dir)
        this->ship.dir -= (HEADING_WEIGHT * new_dir);

    return;
}
