//-Includes------------------------------------------------------------------//

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

QGC_LOGGING_CATEGORY(ShipLandingLog, "ShipLandingLog")

//-Local defines-------------------------------------------------------------//
// Timer interval in seconds, indices by SHIP_LANDING_STATE
const QList<int> TMR_INTVL({30, 15, 10, 5, 5, 2, 1, 1, 1, 1});   //!< timer intervall list of state

// WP-List 0: Loiter, 1: DownToAlt, 2: WP behind ship, 3: WP in front of ship
const QList<double> WPLIST_DIST({400, 300, 100, -100});         //!< distance plane to ship
const QList<unsigned int> WPLIST_ALT({50, 15, 5, 5});           //!< altitude (relative)
const QList<unsigned int> WPLIST_ACCEPT_RAD({15, 10, 5, 1});    //!< acceptance radius for the waypoint
const double GEOFENCE_ANGLE_NET = 180;      //!< geofence angle next to net
const double GEOFENCE_ANGLE_LOITER = 25;    //!< geofence angle behind ship
const double FALLBACK_DIST = -100;      //!< distance plane to ship
const unsigned int FALLBACK_ALT = 200;  //!< altitude (relative)
const int FALLBACK_HDG = 45;            //!< heading relative to ship
const QList<int>FAILSAFE_DIST({75, 50, 25, 10});      //!< min distance plane to ship

// Distance in meter
const double MAX_DISTANCE = 500;        //!< Maximum distance plane to ship
const double NET_DISTANCE = 3/2;        //!< Distance ship position to end of net
const int    MAX_HOR_DIST = 150;        //!< Maximum horizontal distance to ship for start of landing approach
const int    MAX_VERT_DIST = -int(MAX_DISTANCE); //!< Maximum vertical distance to ship for start of landing approach
const int    MIN_VERT_DIST = -int(WPLIST_DIST.at(0)); //!< Minimum vertical distance to ship for start of landing approach

// Calculation parameters for distance vs coordinates
const unsigned int GAP_LATITUDE = 111300;   //!< gap between circles of latitude
const double DEGREE_TO_RAD = M_PI / 180;    //!< conversion degree to radian

// Cartesian Coordinate System: x-axis with 0 degree going counterclockwise
// Heading System: NORTH (y-axis) with 0 degress going clockwise
const double HEADING_WEIGHT = 0.1;                          //!< heading weight
const int HEADING_HIS_SIZE = 10;    //!< Heading history size
const int NORTH = 0, EAST = 90, SOUTH = 180, WEST = 270;    //!< heading angles
const int MAX_HDNG_DIFF = 10;       //!< maximum acceptable heading difference
const int MAX_HDNG_RATE = 2;        //!< maximum acceptable heading change in degrees per second

// Default position
const double HSA_ETECH_LAT = 48.354772;
const double HSA_ETECH_LON = 10.904693;

//-Public functions----------------------------------------------------------//

QObject* ShipLanding::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine);
    Q_UNUSED(scriptEngine);

    qCDebug(ShipLandingLog) << "qmlInstance: Start of ShipLanding";

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

//-Private functions---------------------------------------------------------//

ShipLanding::ShipLanding(QObject *parent) : QObject(parent)
{
    // Position of HSA Etech
    ship.dir = 0;
    ship.coord.setLatitude(HSA_ETECH_LAT);
    ship.coord.setLongitude(HSA_ETECH_LON);

    // Connect GQCPositionManager to USB_GPS.
    connect(qgcApp()->toolbox()->qgcPositionManager(),
            &QGCPositionManager::positionInfoUpdated,
            this, &ShipLanding::update_posShip);

    // Initialize the timerObserve
    timerObserve->setSingleShot(false);
    connect(timerObserve, &QTimer::timeout, this, &ShipLanding::observe_state);

    state = IDLE;
    landReq = false;
    landCancel = false;
}
ShipLanding::~ShipLanding()
{
    disconnect(qgcApp()->toolbox()->qgcPositionManager(),
               &QGCPositionManager::positionInfoUpdated,
               this, &ShipLanding::update_posShip);
    disconnect(timerObserve, &QTimer::timeout, this, &ShipLanding::observe_state);
    timerObserve->stop();
    delete timerObserve;
}

QGeoCoordinate ShipLanding::calcPosRelativeToShip
(double distance, unsigned int altitude, double dDir=0)
{
    QGeoCoordinate pos;
    double dx = sin((ship.dir+dDir) * DEGREE_TO_RAD) * distance;
    double dy = cos((ship.dir+dDir) * DEGREE_TO_RAD) * distance;

    pos.setLatitude(ship.coord.latitude() - dy / GAP_LATITUDE);
    pos.setLongitude(ship.coord.longitude() - dx / GAP_LATITUDE *
                     cos(ship.coord.latitude() * DEGREE_TO_RAD));
    pos.setAltitude(altitude);

    qCDebug(ShipLandingLog) << "calcPosRelativeToShip: pos: " << pos
                            << " | dir: " << ship.dir+dDir
                            << " | dx: " << dx
                            << " | dy: " << dy
                            << " | distanceTo ship: "
                            << ship.coord.distanceTo(pos);
    return pos;
}

_Distance ShipLanding::calcDistanceRelativeTo(double x_p, double y_p, double hdg,
                                              double x_a, double y_a)
{
    _Distance dist;
    double x_u, y_u, r;

    /* Step one: Calculate the heading as a vector. */
    if (hdg < WEST) // first quadrant
    {
        x_u = sin(hdg);
        y_u = cos(hdg);
    }
    else if (hdg < SOUTH) // second quadrant
    {
        hdg -= 90;
        x_u = cos(hdg);
        y_u = -sin(hdg);
    }
    else if (hdg < WEST) // third quadrant
    {
        hdg -= 180;
        x_u = -sin(hdg);
        y_u = -cos(hdg);
    }
    else //fourth quadrant
    {
        hdg -= 270;
        x_u = -cos(hdg);
        y_u = sin(hdg);
    }

   /*
    * Step two: Now we want to pull the following trick:
    *
    *                      P (Reference object)
    *                      |
    *                      | vertical distance
    *                      | g
    * (other object)       |
    * A--------------------F
    *   horizontal distance
    *
    * Let the reference be at point P and the other object be at point A.
    * Together with it's heading vector u, P forms a straight line g, which is
    * defined like this: g: p + r * u
    * Here, p is the position vector of P, u is the heading vector, and r is
    * a real factor. Now, we need to find the dropped perpendicular foot of A.
    * Let's call that F. Afterwards, we just need to calculate the distance
    * between A and F. Sounds easy right?
    * The vector connecting A and F, which is f - a, can be defined like that:
    * ( x_p + r*x_u - x_a )
    * ( y_p + r*y_u - y_a )
    * Since that vector has to be perpendicular to u, their scalar product has
    * got to be zero. Put in formula:
    * (x_p + r*x_u - x_a)*x_u + (y_p + r*y_u - y_a)*y_u = 0
    * ... Some rearranging ...
    */
    r = (-x_p*x_u + x_a*x_u - y_p*y_u + y_a*y_u) / (pow(x_u, 2) + pow(y_u, 2));

    /* Step three: Calculate length of vector f - a */
    dist.horizontal_distance =
                    sqrt(pow(x_p + r*x_u - x_a, 2) + pow(y_p + r*y_u - y_a, 2));

    /* Step four: Calculate length of vector f - p */
    dist.vertical_distance = sqrt(pow(r*x_u, 2) + pow(r*y_u, 2));

    /* Step five: Return :) */
    return dist;
}

double ShipLanding::calcHeadingRate()
{
   /*IMPORTANT NOTE*/
   /*
    * The heading rate (or Rate-of-Turn, as called in nautical science) can
    * also come from good heading sensors and is available in the NMEA2000
    * protocol. This means that this function may have to be replaced by
    * something that is able to extract this information from the NMEA protocol
    * and only uses this implementation in case that there is no available
    * information.
    */
    double heading_rate = 0;
    double xa, ya, xsum = 0, ysum = 0; //x average, y average, x sum, y sum
    double up = 0, down = 0;

    // calculate averages first
    for (unsigned int i = 0; i < ship.hdng_his.size(); i++)
    {
        xsum += ship.hdng_his.at(i).timestamp.toSecsSinceEpoch();
        ysum += ship.hdng_his.at(i).heading;
    }
    xa = xsum / ship.hdng_his.size();
    ya = ysum / ship.hdng_his.size();

    // now, calculate what's on the fracture
    for (unsigned int i = 0; i < ship.hdng_his.size(); i++)
    {
        up += (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa)
                                           * (ship.hdng_his.at(i).heading - ya);
        down += (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa)
                      * (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa);
    }

    if (down != 0)
        heading_rate = up / down;

    return heading_rate;
}
double ShipLanding::calcHeadingDiff()
{
    return fabs(ship.dir - dir_miss);
}

bool ShipLanding::checkPlanePos()
{
    // Calculate distance of plane relative to ship.
    _Distance distance =
        calcDistanceRelativeTo(ship.coord.longitude(), ship.coord.latitude(),
                               ship.dir,
                               _vehicle->coordinate().longitude(),
                               _vehicle->coordinate().latitude());

   /* If the plane isn't in a specific window behind the ship, don't try to
    * land!
    * The < and > are correct the way they are. This is because the vertical
    * distance to ship should be negative. Smaller than MAX_VERT_DIST means
    * further away from the ship, greater than MIN_VERT_DIST means too close
    * to the ship.
    */
    if (fabs(distance.horizontal_distance) > MAX_HOR_DIST  ||
        distance.vertical_distance   < MAX_VERT_DIST ||
        distance.vertical_distance   > MIN_VERT_DIST)
        return false;
    else
        return true;
}
double ShipLanding::checkShipPosDif()
{
    double x_p = _vehicle->coordinate().longitude(),
           y_p = _vehicle->coordinate().latitude();
    double hdg = _vehicle->heading()->rawValue().toDouble();
    double x_a = ship.coord.longitude(), y_a = ship.coord.latitude();

    return calcDistanceRelativeTo(x_p, y_p, hdg, x_a, y_a).horizontal_distance;
}

//-Public slots--------------------------------------------------------------//

void ShipLanding::landingInit()
{
    qCDebug(ShipLandingLog) << "landingInit: Init Landing, ";
    timerObserve->start(TMR_INTVL.at(IDLE) * 1000);
    observe_state();
}
void ShipLanding::landingStart()
{
    qCDebug(ShipLandingLog) << "landingStart: Confirmed Landing";
    landCancel = false;
    landReq = true;
    observe_state();
}
void ShipLanding::landingCancel()
{
    qCDebug(ShipLandingLog) << "landingCancel: Canceled Landing";
    landReq = false;
    landCancel = true;
    observe_state();
}

//-Private Slots-------------------------------------------------------------//

void ShipLanding::send_homePoint()
{
    qCDebug(ShipLandingLog) << "send_homePoint: Send the home point.";
    if(_vehicle)
    {
        QGeoCoordinate newHome = calcPosRelativeToShip(WPLIST_DIST.at(0), WPLIST_ALT.at(0));
        _vehicle->sendMavCommand(_vehicle->defaultComponentId(),
                                 MAV_CMD_DO_SET_HOME, true,
                                 0,              // specified location
                                 0, 0, 0,        // unused param 2-4
                                 static_cast<float>(newHome.latitude()),
                                 static_cast<float>(newHome.longitude()),
                                 static_cast<float>(newHome.altitude()));
    }
}
void ShipLanding::send_fallbackGoAround()
{
    qCDebug(ShipLandingLog) << "send_FallbackGoToPoint: Send the fallback goto point.";
    _vehicle->guidedModeGotoLocation
            (calcPosRelativeToShip(FALLBACK_DIST, FALLBACK_ALT, FALLBACK_HDG));
}
void ShipLanding::send_geofence()
{
    qCDebug(ShipLandingLog) << "send_geofence: Build and send the landing geofence.";

    // Bei Geofence -> ReturnMode (GF_ACTION: 3, s.a. Checklist Failure)
    QmlObjectListModel polygons, circles;
    QGCFencePolygon polygon = new QGCFencePolygon(true);
    QGeoCoordinate breach;
    polygon.appendVertex(calcPosRelativeToShip(NET_DISTANCE, WPLIST_ALT.at(1),
                                               GEOFENCE_ANGLE_NET));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0),
                                               GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0),
                                               -GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToShip(-NET_DISTANCE, WPLIST_ALT.at(1),
                                               -GEOFENCE_ANGLE_NET));
    polygons.insert(0, &polygon);
    _vehicle->geoFenceManager()->sendToVehicle(breach, polygons, circles);
}
void ShipLanding::send_landMission()
{
    qCDebug(ShipLandingLog) << "send_landMission: Build and send the landing mission.";
    QList<QGeoCoordinate> wp;
    for (int i=1; i<WPLIST_DIST.count(); i++)
    {
        wp.push_back(calcPosRelativeToShip(WPLIST_DIST.at(i), WPLIST_ALT.at(i)));
    }
    qCDebug(ShipLandingLog) << "landSend: WP-ist=" << wp;

    // Convert Waypoints into MissionItems
    QList<MissionItem*> landingItems;
    for (int i=1; i<wp.count(); i++)
    {
        landingItems.push_back
                (new MissionItem(i,       // sequence number
                                 MAV_CMD_NAV_WAYPOINT,      // command
                                 MAV_FRAME_GLOBAL_RELATIVE_ALT,          // frame
                                 0,                         // param1 hold time
                                 WPLIST_ACCEPT_RAD.at(i),   // param2 acceptance radius
                                 0,                         // param3 pass trough
                                 std::numeric_limits<int>::quiet_NaN(),     // param4 yaw angle
                                 wp.at(i).latitude(),       // param5 latitude
                                 wp.at(i).longitude(),      // param6 longitude
                                 wp.at(i).altitude(),       // param7 altitude
                                 true,                      // autoContinue
                                 i==1));                    // is current Item
        qCDebug(ShipLandingLog) << "landSend: " << i << "=" << landingItems.back();
    }
    _vehicle->missionManager()->writeMissionItems(landingItems);
}

bool ShipLanding::check_planeNearHomePoint()
{
    if (_vehicle->coordinate().distanceTo(ship.coord) > MAX_DISTANCE ||
            _vehicle->coordinate().distanceTo(ship.coord) < WPLIST_DIST.at(1))
        return false;
    else return true;
}
bool ShipLanding::check_shipHeadingRate()
{
    if (calcHeadingRate() > MAX_HDNG_RATE)
        return false;
    else return true;
}
bool ShipLanding::check_shipHeadingDifference()
{
    if (calcHeadingDiff() > MAX_HDNG_DIFF)
        return false;
    else return true;
}
bool ShipLanding::check_shipDroveOverLastWP()
{
    // ship passes last wp formerly in front of ship
    return true;
}
bool ShipLanding::check_maxDistShipToPlane()
{
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DISTANCE)
        return false;
    else return true;
}

void ShipLanding::observe_state()
{
    if (state != IDLE && !check_maxDistShipToPlane())
        send_homePoint();

    switch (state) {
    case IDLE:
        qCDebug(ShipLandingLog) << "observeState: IDLE";
        if(_vehicle->missionManager()->currentIndex() > 0 || _vehicle->flying())
            state = MISSION;
        break;
    case MISSION:
        qCDebug(ShipLandingLog) << "observeState: MISSION";
        if (landReq == true)
            state = LAND_REQ;
        else if(_vehicle->missionManager()->currentIndex() >= _vehicle->missionManager()->PlanManager::missionItems().count() - 1 ||
                _vehicle->flightMode().compare(_vehicle->rtlFlightMode(), Qt::CaseInsensitive) == 0)
            state = RETURN;
        break;
    case RETURN:
        qCDebug(ShipLandingLog) << "observeState: RETURN";
        if (!check_maxDistShipToPlane()  && _vehicle->flightMode().compare(_vehicle->rtlFlightMode(), Qt::CaseInsensitive) != 0)
            _vehicle->guidedModeRTL();
        if (landReq)
            state = LAND_REQ;
        break;
    case LAND_REQ:
        qCDebug(ShipLandingLog) << "observeState: LAND_REQ";
        if (!check_maxDistShipToPlane())
            _vehicle->guidedModeRTL();
        if (landCancel)
            state = RETURN;
        else if (check_planeNearHomePoint() && check_shipHeadingRate())
            state = LAND_SEND;
        break;
    case LAND_SEND:
        qCDebug(ShipLandingLog) << "observeState: LAND_SEND";
        send_landMission();
        send_geofence();
        _vehicle->startMission();
        _vehicle->_setLanding(true);
        if (landCancel)
        {
            _vehicle->_setLanding(false);
            state = RETURN;
        }
        else
            state = LAND_APPROACH;
        break;
    case LAND_APPROACH:
        qCDebug(ShipLandingLog) << "observeState: LAND_APPROACH";
        if (checkPlanePos() || check_shipHeadingDifference())
        {
            double dist = ship.coord.distanceTo(_vehicle->coordinate());
            if (dist > FAILSAFE_DIST.at(0))
                state = FALLBACK_RESTART_APPROACH;
            else if (dist > FAILSAFE_DIST.at(1))
                state = FALLBACK_RESTART_LOITER;
            else if (dist > FAILSAFE_DIST.at(2))
                state = FALLBACK_GO_AROUND;
            else if (dist > FAILSAFE_DIST.at(3))
                state = FALLBACK_PNR;
        }
        else if (landCancel)
        {
            _vehicle->_setLanding(false);
            state = RETURN;
        }
        else if (check_shipDroveOverLastWP())
            state = FALLBACK_RESTART_APPROACH;
        break;
    case FALLBACK_RESTART_APPROACH:
        qCDebug(ShipLandingLog) << "observeState: FALLBACK_RESTART_APPROACH";
        state = LAND_SEND;
        break;
    case FALLBACK_RESTART_LOITER:
        qCDebug(ShipLandingLog) << "observeState: FALLBACK_RESTART_LOITER";
        _vehicle->_setLanding(false);
        state = LAND_REQ;
        landReq = true;
        break;
    case FALLBACK_GO_AROUND:
        qCDebug(ShipLandingLog) << "observeState: FALLBACK_GO_AROUND";
        send_fallbackGoAround();
        _vehicle->_setLanding(false);
        state = RETURN;
        landReq = false;
        break;
    case FALLBACK_PNR:
        qCDebug(ShipLandingLog) << "observeState: FALLBACK_PNR";
        if (landCancel)
        {
            _vehicle->_setLanding(false);
            state = RETURN;
        }
        break;
    }

    timerObserve->setInterval(TMR_INTVL.at(state) * 1000);
    qCDebug(ShipLandingLog) << "observeState: newState=" << state << "timer=" << TMR_INTVL.at(state);
}

void ShipLanding::update_posShip(QGeoPositionInfo update)
{
    // Safe the old GPS data to calculate heading. If NaN pos is HSA Etech.
    QGeoCoordinate old_ship_pos = ship.coord;
    if (qIsNaN(ship.coord.longitude()) || qIsNaN(ship.coord.latitude()))
    {
        qCDebug(ShipLandingLog) << "update_posShip: Old ship pos is Nan.";
        old_ship_pos.setLongitude(HSA_ETECH_LAT);
        old_ship_pos.setLatitude(HSA_ETECH_LON);
    }

    // Transfer GPS data to struct
    ship.coord = update.coordinate();

   /*
    * Calculate heading. Replace later by values from GPS sensor.
    * Idea: Start heading with 0 degrees.
    * Afterwards, calculate a heading from old and new GPS.
    * Add this heading to the new one, but weighed in with a factor.
    * Please note that this won't work at 180 degrees longitude, but we are at
    * mediteranian sea and this is supposed to be a workaround anyway, so I
    * won't bother building this to work at 180 degrees.
    */
    double alpha;   // Alpha shows the angle in a normal coordinate system.
    double new_dir; // New direction.

    // Check for  0 degree and 180 degree (s.a. atan!)
    if (ship.coord.longitude() == old_ship_pos.longitude())
    {
        // heading didn't change
        if (ship.coord.latitude() == old_ship_pos.latitude())
            new_dir = ship.dir;
        else if (ship.coord.latitude() > old_ship_pos.latitude())
            new_dir = NORTH;    // 0 degree
        else
            new_dir = SOUTH;    // 180 degree
    }
    else
    {
        alpha = atan((ship.coord.latitude() - old_ship_pos.latitude())/
                     (ship.coord.longitude() - old_ship_pos.longitude()));

        // 1. and 2. quadrant: alpha > 0: 1.Qdr, alpha < 0: 2.Qdr
        if (ship.coord.longitude() > old_ship_pos.longitude())
            new_dir = EAST - alpha;
        // 3. and 4. quadrant: alpha > 0: 3.Qdr, alpha < 0: 4.Qdr
        else
            new_dir = WEST - alpha;
    }

    ship.dir = ship.dir + (HEADING_WEIGHT * (new_dir - ship.dir));

    // Put new direction in history queue
    _Heading new_entry = {ship.dir, update.timestamp()};
    ship.hdng_his.push_back(new_entry);
    if (ship.hdng_his.size() > HEADING_HIS_SIZE)
        ship.hdng_his.pop_front();

    return;
}
