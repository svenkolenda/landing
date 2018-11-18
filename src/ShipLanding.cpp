/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

/*-Local defines--------------------------------------------------------------*/
const bool UNITTEST = false;
const double float_epsilon = 1e-3;

// Distance in meter, Time in seconds
const int MAX_DISTANCE = 200;      //!< Maximum distance plane to ship
const int LOITER_DISTANCE = 150;   //!< Distance plane to ship for loiter point
const int LOITER_ALTITUDE = 50;    //!< Altitude (absolute) of the loiter point
const int LOITER_UPDATE = 30;      //!< Timer intervall to check loiter
const int WP2_DISTANCE = 100;      //!< Distance plane to ship for wp2
const int WP2_ALTITUDE = 25;       //!< Altitude (absolute) for wp2
const int WP3_DISTANCE = 50;       //!< Distance plane to ship for wp3
const int WP3_ALTITUDE = 5;        //!< Altitude (absolute) for wp3
const int WP4_DISTANCE = -100;     //!< Distance plane to ship for wp4
const int WP4_ALTITUDE = 5;        //!< Altitude (absolute) for wp4

const double HEADING_WEIGHT = 0.1;

/*
 * The following are necessary because the heading doesn't work like angles in
 * a normal coordinate system (starting at the x-axis with 0 degrees going
 * counterclockwise). Instead, it starts at NORTH (y-axis), going clockwise.
 */

const int NORTH = 0; //degrees
const int EAST = 90; //degrees
const int SOUTH = 180; //degrees
const int WEST = 270; //degrees

/*-Public functions-----------------------------------------------------------*/

QObject* ShipLanding::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine);
    Q_UNUSED(scriptEngine);

    if (_instance == nullptr)
        _instance = new ShipLanding();
    return _instance;
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
    qDebug() << "Confirmed Landing";
    stop_timerLoiter();                     // stop timer to cancel loiter-check
    emit confirmLanding();
}

void ShipLanding::cancelDialog()
{
    qDebug() << "Confirmed Cancel";
    start_timerLoiter();
    emit confirmCancel();
}

/*-Private functions----------------------------------------------------------*/

ShipLanding::ShipLanding(QObject *parent) : QObject(parent)
{
    // Connect our signals to corresponding functions
    connect(this, &ShipLanding::confirmLanding, this, &ShipLanding::land);

    // TODO: Connect GQCPositionManager to USB_GPS.
    connect(qgcApp()->toolbox()->qgcPositionManager(),
            &QGCPositionManager::positionInfoUpdated,
            this,
            &ShipLanding::update_posPlane);

    // TODO: Position plane.
    /* connect(qgcApp()->toolbox()->PositionManager(),
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

QGeoCoordinate ShipLanding::calcPosRelativeToShip
(int distance, unsigned int altitude)
{
    if (UNITTEST)
    {
        ship.dir = 225;
        ship.coord.setAltitude(500);
        ship.coord.setLatitude(47.4065160);
        ship.coord.setLongitude(8.5425730);
    }

    /* TODO: Get rid of the magic numbers. */
    QGeoCoordinate pos;
    double dx = 0, dy = 0;
    const unsigned int gapLatitude = 111300;
    const double degreeToRad = M_PI / 180;

    dx = sin(ship.dir * degreeToRad) * distance;
    dy = cos(ship.dir * degreeToRad) * distance;

    pos.setLatitude(ship.coord.latitude() - dy / gapLatitude);
    pos.setLongitude(ship.coord.longitude() - dx / gapLatitude *
                     cos(ship.coord.latitude() * degreeToRad));
    pos.setAltitude(altitude);

    if (UNITTEST)
        qDebug() << "Longitude: " << pos.longitude()
                 << " | Latitude: " << pos.latitude()
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

void ShipLanding::loiterShip()
{
    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    // Check distance plane - ship too far
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DISTANCE)
    {
        // Send the plane distance behind the ship
        if(_vehicle)
            _vehicle->guidedModeGotoLocation(calcPosRelativeToShip(LOITER_DISTANCE, LOITER_ALTITUDE));
    }
}

void ShipLanding::land()
{
    //Get vehicle
    Vehicle* vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();

    // 2. WP: Loiter down to goal height
    QGeoCoordinate wp2 = calcPosRelativeToShip(WP2_DISTANCE, WP2_ALTITUDE);

    // 3. WP: Before Net
    QGeoCoordinate wp3 = calcPosRelativeToShip(WP3_DISTANCE, WP3_ALTITUDE);

    // 4. WP: Behind ship so that we won't loiter
    QGeoCoordinate wp4 = calcPosRelativeToShip(WP4_DISTANCE, WP4_ALTITUDE);

    //vehicle->missionManager()->writeMissionItems(missionItems);

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
    QGeoCoordinate old_ship_pos = ship.coord;

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

    if (fabs(new_ship_pos.longitude() - old_ship_pos.longitude()) < float_epsilon)
    {
       /*
        * We cannot calculate alpha in this case, because we would divide by
        * zero.
        */
        if (fabs(new_ship_pos.latitude() - old_ship_pos.latitude()) < float_epsilon)
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

    ship.dir = ship.dir + (HEADING_WEIGHT * (new_dir - ship.dir));
}
