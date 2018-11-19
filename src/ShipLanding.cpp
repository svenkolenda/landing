/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

/*-Local defines--------------------------------------------------------------*/
const bool UNITTEST = false;

// Distance in meter, Time in seconds
const int MAX_DISTANCE = 200;      //!< Maximum distance plane to ship
const int LOITER_DISTANCE = 150;   //!< Distance plane to ship for loiter point
const int LOITER_ALTITUDE = 50;    //!< Altitude (absolute) of the loiter point
const int LOITER_UPDATE = 30;      //!< Timer intervall to check loiter
const int WP2_DISTANCE = 100;      //!< Distance plane to ship for wp2
const int WP2_ALTITUDE = 25;       //!< Altitude (absolute) for wp2
const int WP3_DISTANCE = 50;       //!< Distance plane to ship for wp3
const int WP3_ALTITUDE = 5;        //!< Altitude (absolute) for wp3
const int WP4_DISTANCE = -50;      //!< Distance plane to ship for wp4
const int WP4_ALTITUDE = 5;        //!< Altitude (absolute) for wp4
const double WP_ACCEPT_RADIUS = 5; //!< Acceptance radius for the waypoint
const double WP_PASS_TROUGH = 0;   //!< plane pass through the waypoint
const double WP_LOITER_TIME = 0;   //!< Hold time at the waypoint
const int WP_SEQ_NR = 13;          //!< Sequence number
const int OBSERVE_UPDATE = 30;     //!< Timer intervall to observe landing

const double HEADING_WEIGHT = 0.1; //!< heading weight
const unsigned int GAP_LATITUDE = 111300;
const double DEGREE_TO_RAD = M_PI / 180;

/*
 * The following are necessary because the heading doesn't work like angles in
 * a normal coordinate system (starting at the x-axis with 0 degrees going
 * counterclockwise). Instead, it starts at NORTH (y-axis), going clockwise.
 */

const int NORTH = 0;
const int EAST = 90;
const int SOUTH = 180;
const int WEST = 270;

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
    stop_timerLoiter();                 // stop timer to loiter
    start_timerObserve();               // start timer to observe landing
    landObserve();
}

void ShipLanding::cancelDialog()
{
    qDebug() << "Confirmed Cancel";
    stop_timerObserve();                // stop timer to observe landing
    start_timerLoiter();                // start timer to loiter
    loiterSend();
}

/*-Private functions----------------------------------------------------------*/

ShipLanding::ShipLanding(QObject *parent) : QObject(parent)
{
    // TODO: Connect GQCPositionManager to USB_GPS.
    connect(qgcApp()->toolbox()->qgcPositionManager(),
            &QGCPositionManager::positionInfoUpdated,
            this, &ShipLanding::update_posPlane);


    // Initialize the timerLoiter
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterSend);
    // Initialize the timerObserve
    timerObserve->setSingleShot(false);
    connect(timerObserve, &QTimer::timeout, this, &ShipLanding::landObserve);

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

    QGeoCoordinate pos;
    double dx = 0, dy = 0;

    dx = sin(ship.dir * DEGREE_TO_RAD) * distance;
    dy = cos(ship.dir * DEGREE_TO_RAD) * distance;

    pos.setLatitude(ship.coord.latitude() - dy / GAP_LATITUDE);
    pos.setLongitude(ship.coord.longitude() - dx / GAP_LATITUDE *
                     cos(ship.coord.latitude() * DEGREE_TO_RAD));
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

void ShipLanding::start_timerObserve()
{
    timerObserve->start(OBSERVE_UPDATE * 1000);
}

void ShipLanding::stop_timerObserve()
{
    timerLoiter->stop();
}

/*-Private Slots--------------------------------------------------------------*/

void ShipLanding::loiterSend()
{
    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()
                                                              ->activeVehicle();
    // Check distance plane - ship too far
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DISTANCE)
    {
        // Send the plane distance behind the ship
        if(_vehicle)
            _vehicle->guidedModeGotoLocation
                      (calcPosRelativeToShip(LOITER_DISTANCE, LOITER_ALTITUDE));
    }
}

void ShipLanding::landSend()
{
    //Get vehicle
    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()
                                                              ->activeVehicle();

    QList<QGeoCoordinate> wp;
    // WP0: Down to altitude
    wp.push_back(calcPosRelativeToShip(WP2_DISTANCE, WP2_ALTITUDE));
    // WP1: Behind ship in front of the net
    wp.push_back(calcPosRelativeToShip(WP3_DISTANCE, WP3_ALTITUDE));
    // WP2: In front of the ship
    wp.push_back(calcPosRelativeToShip(WP4_DISTANCE, WP4_ALTITUDE));

    // Convert Waypoints into MissionItems
    QList<MissionItem*> landingItems;
    for (int i=0; i<wp.count(); i++) {
        landingItems.push_back
            (new MissionItem(WP_SEQ_NR,               // sequence number
             MAV_CMD_NAV_WAYPOINT,    // command
             MAV_FRAME_GLOBAL,        // frame
             WP_LOITER_TIME,          // param1 hold time
             WP_ACCEPT_RADIUS,        // param2 acceptance radius
             WP_PASS_TROUGH,          // param3 pass trough
             std::numeric_limits<int>::quiet_NaN(),   // param4 yaw angle
             wp.at(i).latitude(),     // param5 latitude
             wp.at(i).longitude(),    // param6 longitude
             wp.at(i).altitude(),     // param7 altitude
             true,                    // autoContinue
             i==0));                  // is current Item
    }
    //vehicle->missionManager()->writeMissionItem(landingItems);

    // Bei Übertritt Geofence -> ReturnMode (Parameter GF_ACTION auf 3 setzen PX4MockLink.params, V1.4.OfflineEditing.params)
    //QList<MissionItem*> fenceItems;
    //QmlObjectListModel polygons, circles;
    //TODO: Create polygons VS. Create MissionItem s.a. GeofenceManager
    //vehicle->geoFenceManager()->sendToVehicle(wp.at(0), polygons, circles);
}

void ShipLanding::landObserve()
{
    // TODO: Watch ship heading rate of change

    // TODO: Watch ship position relative to wp2 (in front of ship)

    // TODO: Check for current waypoint

    // TODO: if ...
    landSend();
}

void ShipLanding::update_posPlane()
{
    // Transfer GPS data to struct

    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()
                                                              ->activeVehicle();
    plane.coord = _vehicle->coordinate();

    if (UNITTEST)
        qDebug() << plane.coord.altitude() << "||" << plane.coord.latitude()
                                           << "||" << plane.coord.longitude();
}

void ShipLanding::update_posShip(QGeoPositionInfo update)
{
   /*
    * Safe away the old GPS data to calculate heading later.
    */
    QGeoCoordinate old_ship_pos = ship.coord;

    // Transfer GPs data to struct
     ship.coord = update.coordinate();
     if (UNITTEST)
        qDebug() << ship.coord.altitude() << "||" << ship.coord.latitude()
                                          << "||" << ship.coord.longitude();

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

    ship.dir = ship.dir + (HEADING_WEIGHT * (new_dir - ship.dir));
}
