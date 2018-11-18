/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

/*-Local defines--------------------------------------------------------------*/
#ifndef TEST
#define TEST 1
#endif
static const bool UNITTEST = false;

// Distance in meter, Time in seconds
const unsigned int MAX_DISTANCE = 200;      //!< Maximum distance plane to ship
const unsigned int LOITER_DISTANCE = 150;   //!< Distance plane to ship for loiter point
const unsigned int LOITER_ALTITUDE = 50;    //!< Altitude (absolute) of the loiter point
const unsigned int LOITER_UPDATE = 30;      //!< Timer intervall to check loiter
const unsigned int WP2_DISTANCE = 100;      //!< Distance plane to ship for wp2
const unsigned int WP2_ALTITUDE = 25;       //!< Altitude (absolute) for wp2
const unsigned int WP3_DISTANCE = 50;       //!< Distance plane to ship for wp3
const unsigned int WP3_ALTITUDE = 5;        //!< Altitude (absolute) for wp3
const unsigned int WP4_DISTANCE = 0;        //!< Distance plane to ship for wp4
const unsigned int WP4_ALTITUDE = 5;        //!< Altitude (absolute) for wp4

static const double HEADING_WEIGHT = 0.1;

/*
 * WARNING:
 * IF YOU CHANGE ANY OF THOSE, YOU BETTER MAKE SURE THAT THEY STILL FIT IN THE
 * GIVEN DATATYPE!
 */

//static const uint16_t MAX_DISTANCE = 300;      // meter
//static const uint8_t LOITER_DISTANCE = 100;    // meter
//static const uint8_t LOITER_UPDATE = 10;               // second
//static const uint8_t LOITER_ALTITUDE = 50;             // meter

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

QObject* ShipLanding::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine);
    Q_UNUSED(scriptEngine);

    if (_instance == nullptr)
        _instance = new ShipLanding();
    return _instance;;
}

void ShipLanding::release()
{
    if (_instance != nullptr)
        delete _instance;
    _instance = nullptr;
}

/*-Public slots---------------------------------------------------------------*/

void ShipLanding::initDialog()
{
#ifdef TEST
    qDebug() << "Init Dialog yes";
#endif
    emit confirmLanding();
}

void ShipLanding::cancelDialog()
{
#ifdef TEST
    qDebug() << "Cancel Dialog yes";
#endif
    emit confirmCancel();
}

/*-Private functions----------------------------------------------------------*/

ShipLanding::ShipLanding(QObject *parent) : QObject(parent)
{
    // Connect our signals to corresponding functions
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

    // Initialize the timerLoiter
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterShip);

    // TODO: Connect prepare to land to end of mission
}

ShipLanding::~ShipLanding()
{
}

QGeoCoordinate ShipLanding::calcPosRelativeToShip(unsigned int distance, unsigned int alltitude)
{
    // TODO: Choose one!
#if TEST == 1
    ship.dir = 225;
    ship.coord.setAltitude(500);
    ship.coord.setLatitude(47.4065160);
    ship.coord.setLongitude(8.5425730);
#endif
    if (UNITTEST)
    {
        ship.dir =225;
        ship.coord.setAltitude(500);
        ship.coord.setLatitude(47.4065160);
        ship.coord.setLongitude(8.5425730);
    }

    QGeoCoordinate pos;
    double dx = 0, dy = 0;
    const unsigned int gapLatitude = 111300;
    const double degreeToRad = M_PI / 180;

    dx = sin(ship.dir * degreeToRad) * distance;
    dy = cos(ship.dir * degreeToRad) * distance;

    pos.setLatitude(ship.coord.latitude() - dy / gapLatitude);
    pos.setLongitude(ship.coord.longitude() - dx / gapLatitude * cos(ship.coord.latitude() * degreeToRad));
    pos.setAltitude(alltitude);

    // TODO: Choose one!
#if TEST == 1
    qDebug() << "Longitude: " << pos.longitude() << " | Latitude: " << pos.latitude()
             << " | dx: " << dx << " | dy: " << dy
             << " | distanceTo ship: " << ship.coord.distanceTo(pos);
#endif
    if (UNITTEST)
        qDebug() << "Longitude: " << pos.longitude() << " | Latitude: " << pos.latitude()
                 << " | dx: " << dx << " | dy: " << dy
                 << " | distanceTo ship: " << ship.coord.distanceTo(pos);
    return pos;
}

void ShipLanding::start_timerLoiter()
{
    timerLoiter->start(LOITER_UPDATE * 1000);
}

void ShipLanding::stop_timerLoiter()
{
    timerLoiter->stop();
}

/*-Private Slots--------------------------------------------------------------*/

void ShipLanding::prepareToLoiter()
{
    /* TODO: Is interfacing this really needed? */
    start_timerLoiter();
}

void ShipLanding::loiterShip()
{
    if (ship.coord.distanceTo(plane.coord) > MAX_DISTANCE)
    {
        Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
        // Send the plane distance behind the ship
        if(_vehicle)
        {
            _vehicle->guidedModeGotoLocation(calcPosRelativeToShip(LOITER_DISTANCE, LOITER_ALTITUDE));
        }
    }
}

void ShipLanding::land()
{
    /* TODO: Is interfacing this really needed? */
    stop_timerLoiter();                     // stop timer to cancel loiter-check

    /*
    * Something something landing
    * Ideas what could happen here:
    *  - mission + second mission in background
    *  - mission + check for missed boundaries via geofence + check for proper height with altitude check
    *  - mission + L1 library
    */

    // 2. WP: Loiter down to goal height
    QGeoCoordinate wp2 = calcPosRelativeToShip(WP2_DISTANCE, WP2_ALTITUDE);

    // 3. WP: Before Net
    QGeoCoordinate wp3 = calcPosRelativeToShip(WP3_DISTANCE, WP3_ALTITUDE);

    // 4. WP: Behind ship so that we won't loiter
    QGeoCoordinate wp4 = calcPosRelativeToShip(WP4_DISTANCE, WP4_ALTITUDE);

    qgcApp()->toolbox()->multiVehicleManager()->activeVehicle()->missionManager()->writeMissionItems(missionItems);

    // After 4. WP: Go back to WP2 and start again (some height)

    // Erweiterung: Beurteilung - Schaffe ich es noch? Ab Headingsänderung so und so lade neue Mission hoch
    // Bei Übertritt Geofence -> Schräg hochziehen

    // TODO: Point of no return reached -> User cannot stop this!
    // Left geofence -> return behind ship

    // TODO: Failsafe prepareToLoiter
}

void ShipLanding::update_posPlane()
{
    // Transfer GPS data to struct
    // TODO: QGCPositionManager => __GPS
}

void ShipLanding::update_posShip()
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

    alpha = atan((new_ship_pos.latitude() - old_ship_pos.latitude())/
                 (new_ship_pos.longitude() - old_ship_pos.longitude()));

    if (new_ship_pos.latitude() > old_ship_pos.latitude())
    {
        if (new_ship_pos.longitude() > old_ship_pos.longitude())
            new_dir = EAST - alpha; //alpha is positive, first quadrant
        else
            new_dir = WEST - alpha; //alpha is negative, fourth quadrant
    }
    else
    {
        if (new_ship_pos.longitude() > old_ship_pos.longitude())
            new_dir = WEST - alpha; //alpha is positive, third quadrant
        else
            new_dir = EAST - alpha; //alpha is negative, second quadrant
    }

    this->ship.dir += (HEADING_WEIGHT * new_dir);
}
