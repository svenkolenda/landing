//-Includes---------------------------------------------------------------------------------------//

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

QGC_LOGGING_CATEGORY(ShipLandingLog, "ShipLandingLog")

//-Local defines----------------------------------------------------------------------------------//

// distance in meter [m], time param in second [s], angle in degree [°]

// Timer interval, indices by SHIP_LANDING_STATE
const QList<int> TMR_INTVL({30, 15, 10, 5, 5, 2, 1, 1, 1, 1});   //!< Timer intervall list of state

// WP-List 0: Loiter, 1: DownToAlt, 2: WP behind ship, 3: WP in front of ship
const QList<double> WPLIST_DIST({800, 600, 100, -500, -600});     //!< Distance plane to ship
const QList<unsigned int> WPLIST_ALT({30, 15, 5, 5, 5});          //!< Altitude (absolute)
const QList<unsigned int> WPLIST_ACCEPT_RAD({15, 10, 5, 1, 1});   //!< Acceptance radius for the waypoint
const int WATER_DIST                    = 50;                     //!< Distance plane to water point
const int WATER_ALT                     = 0;                      //!< Altitude of the water point
const int WATER_ANGLE                   = 90;                     //!< Angle ship to water point

// Geofence
const double GEOFENCE_ANGLE_NET     = 180;                   //!< Geofence angle in front of ship
const double GEOFENCE_ANGLE_SHIP    = 90;                    //!< Geofence angle beneath ship
const double GEOFENCE_ANGLE_LOITER  = 25;                    //!< Geofence angle behind ship
const double GEOFENCE_MULTIPLY_DIST = 1.5;                   //!< Multiplication factor of distances
const int GEOFENCE_DIST_CORRIDOR    = 10;                    //!< Geofence distance from ship to corridor

// Fallback Go-Around parameter
const double GO_AROUND_DIST         = -100;                  //!< Distance plane to ship
const unsigned int GO_AROUND_ALT    = 200;                   //!< Altitude (absolute)
const int GO_AROUND_HDG             = 45;                    //!< Heading relative to ship

// Fallback distance
const QList<int>FALLBACK_DIST({100, 50, 25, 10});             //!< Fallback distances plane to ship

// Max and min distances
const double MAX_DIST_PLANE_SHIP = 1000;                //!< Maximum distance plane to ship
const double MIN_DIST_PLANE_SHIP = 600;
const double MIN_DIST_SHIP_WP    = 42;                  //!< Minimum distance ship to last wp
const int MAX_HOR_DIST   = 5;                           //!< Maximum horizontal distance of ship to
                                                        //!< mission trajectorie
const int MAX_DIST_HOME_POS = 100;                      //!< Maximum distance plane to home point

// Calculation parameters for distance vs coordinates
const unsigned int GAP_LATITUDE = 111300;               //!< Gap between circles of latitude
const double DEGREE_TO_RAD      = M_PI / 180;           //!< Conversion degree to radian

// Cartesian Coordinate System: x-axis with 0 degree going counterclockwise
// Heading System: NORTH (y-axis) with 0 degress going clockwise
const double HDNG_WEIGHT     = 0.1;                         //!< Heading weight
const int HDNG_HIS_SIZE      = 10;                          //!< Heading history size
const int NORTH = 0, EAST = 90, SOUTH = 180, WEST = 270;    //!< Heading angles
const int MAX_HDNG_DIFF         = 5;                        //!< Maximum acceptable heading
                                                            //!< difference
const int MAX_HDNG_RATE         = 2;                        //!< Maximum acceptable heading change
                                                            //!< in degrees per second

// Default position
const double HSA_ETECH_LAT = 48.354772;
const double HSA_ETECH_LON = 10.904693;

//-Public functions-------------------------------------------------------------------------------//

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

    return;
}

//-Private functions------------------------------------------------------------------------------//

ShipLanding::ShipLanding(QObject *parent) : QObject(parent)
{
    // Position of HSA Etech
    ship.dir = 0;
    ship.coord.setLatitude(HSA_ETECH_LAT);
    ship.coord.setLongitude(HSA_ETECH_LON);
    water.coord.setLatitude(HSA_ETECH_LAT);
    water.coord.setLongitude(HSA_ETECH_LON);

    // Connect GQCPositionManager to USB_GPS.
    connect(qgcApp()->toolbox()->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated,
                                                                this, &ShipLanding::updatePosShip);

    // Initialize the timerObserve
    timerObserve->setSingleShot(false);
    connect(timerObserve, &QTimer::timeout, this, &ShipLanding::observeState);

    // Reset state machine
    state = IDLE;
    landReq = false;
    landWater = false;
    landCancel = false;
    landReset = false;

    return;
}
ShipLanding::~ShipLanding()
{
    // Disconnect position updates and stop timer
    disconnect(qgcApp()->toolbox()->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated,
                                                                this, &ShipLanding::updatePosShip);
    disconnect(timerObserve, &QTimer::timeout, this, &ShipLanding::observeState);
    timerObserve->stop();
    delete timerObserve;

    return;
}

QGeoCoordinate ShipLanding::calcPosRelativeToCoord
                    (double distance, unsigned int altitude, QGeoCoordinate coord, double dDir = 0)
{
    QGeoCoordinate pos;
    double dx = sin((ship.dir+dDir) * DEGREE_TO_RAD) * distance;
    double dy = cos((ship.dir+dDir) * DEGREE_TO_RAD) * distance;

    pos.setLatitude(coord.latitude() - dy / GAP_LATITUDE);
    pos.setLongitude(coord.longitude() - dx
                                       / GAP_LATITUDE * cos(coord.latitude() * DEGREE_TO_RAD));
    pos.setAltitude(altitude);

    qCDebug(ShipLandingLog) << "calcPosRelativeToShip: pos: " << pos
                            << " | dir: " << ship.dir+dDir
                            << " | dx: " << dx
                            << " | dy: " << dy
                            << " | distanceTo ship: "
                            << coord.distanceTo(pos);

    return pos;
}

_Distance ShipLanding::calcDistanceRelativeTo
                                        (double x_p, double y_p, double hdg, double x_a, double y_a)
{
    _Distance dist;
    double x_u, y_u, r;

    // Step one: Calculate the heading as a vector
    if (hdg < WEST)         // first quadrant
    {
        x_u = sin(hdg);
        y_u = cos(hdg);
    }
    else if (hdg < SOUTH)   // second quadrant
    {
        hdg -= 90;
        x_u = cos(hdg);
        y_u = -sin(hdg);
    }
    else if (hdg < WEST)    // third quadrant
    {
        hdg -= 180;
        x_u = -sin(hdg);
        y_u = -cos(hdg);
    }
    else                    //fourth quadrant
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
    * Together with it's heading vector u, P forms a straight line g, which is defined like this:
    * g: p + r * u
    * Here, p is the position vector of P, u is the heading vector, and r is a real factor.
    * Now, we need to find the dropped perpendicular foot of A. Let's call that F.
    * Afterwards, we just need to calculate the distance between A and F. Sounds easy right?
    * The vector connecting A and F, which is f - a, can be defined like that:
    * ( x_p + r*x_u - x_a )
    * ( y_p + r*y_u - y_a )
    * Since that vector has to be perpendicular to u, their scalar product has got to be zero.
    * Put in formula:
    * (x_p + r*x_u - x_a)*x_u + (y_p + r*y_u - y_a)*y_u = 0
    * ... Some rearranging ...
    */
    r = (-x_p*x_u + x_a*x_u - y_p*y_u + y_a*y_u) / (pow(x_u, 2) + pow(y_u, 2));

    // Step three: Calculate length of vector f - a
    dist.horizontal_distance = sqrt(pow(x_p + r*x_u - x_a, 2) + pow(y_p + r*y_u - y_a, 2));

    // Step four: Calculate length of vector f - p
    dist.vertical_distance = sqrt(pow(r*x_u, 2) + pow(r*y_u, 2));

    // Step five: Return :)
    return dist;
}

double ShipLanding::calcHeadingRate()
{
   /*
    * IMPORTANT TODO
    * The heading rate (or Rate-of-Turn, as called in nautical science) can also come from good
    * heading sensors and is available in the NMEA2000 protocol. This means that this function may
    * have to be replaced by something that is able to extract this information from the NMEA
    * protocol and only uses this implementation in case that there is no available
    * information.
    */
    double heading_rate = 0;
    double xa, ya, xsum = 0, ysum = 0; //x average, y average, x sum, y sum
    double up = 0, down = 0;

    // Calculate averages first
    for (unsigned int i = 0; i < ship.hdng_his.size(); i++)
    {
        xsum += ship.hdng_his.at(i).timestamp.toSecsSinceEpoch();
        ysum += ship.hdng_his.at(i).heading;
    }
    xa = xsum / ship.hdng_his.size();
    ya = ysum / ship.hdng_his.size();

    // Now, calculate what's on the fracture
    for (unsigned int i = 0; i < ship.hdng_his.size(); i++)
    {
        up += (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa)
                                                               * (ship.hdng_his.at(i).heading - ya);
        down += (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa)
                                          * (ship.hdng_his.at(i).timestamp.toSecsSinceEpoch() - xa);
    }

    // The warning is here on purpose
    if (down != 0)
        heading_rate = up / down;

    return heading_rate;
}

double ShipLanding::calcHeadingDiff()
{
    return fabs(ship.dir - dir_miss);
}

double ShipLanding::calcShipPosDif()
{
    double x_p = _vehicle->coordinate().longitude(),
           y_p = _vehicle->coordinate().latitude();
    double hdg = _vehicle->heading()->rawValue().toDouble();
    double x_a = ship.coord.longitude(),
           y_a = ship.coord.latitude();

    return calcDistanceRelativeTo(x_p, y_p, hdg, x_a, y_a).horizontal_distance;
}

bool ShipLanding::checkPlaneNearHomePoint()
{
    if (_vehicle->coordinate().distanceTo(_vehicle->homePosition()) > MAX_DIST_HOME_POS)
        return false;
    else return true;
}

bool ShipLanding::checkShipInLandingCorridor()
{
    // Calculate horizontal distance of ship relative to mission trajectorie,
    // here waypoint 3 as referrenced object.
    _Distance distance =
            calcDistanceRelativeTo(ship.coord.longitude(), ship.coord.latitude(),
                                   ship.dir,
                                   wp.at(3).longitude(), wp.at(3).latitude());

    // Check if the ship ist still in landing corridor.
    if (distance.horizontal_distance > MAX_HOR_DIST)
        return false;
    else
        return true;
}

bool ShipLanding::checkWaterInLandingCorridor()
{
    // Calculate horizontal distance of water relative to mission trajectorie,
    // here waypoint 3 as referrenced object.
    _Distance distance =
            calcDistanceRelativeTo(water.coord.longitude(), water.coord.latitude(),
                                   ship.dir,
                                   wp.at(3).longitude(), wp.at(3).latitude());

    // Check if the ship ist still in landing corridor.
    if (distance.horizontal_distance > MAX_HOR_DIST)
        return false;
    else
        return true;
}

bool ShipLanding::checkShipHeadingRate()
{
    if (calcHeadingRate() > MAX_HDNG_RATE)
        return false;
    else return true;
}

bool ShipLanding::checkShipHeadingDifference()
{
    if (calcHeadingDiff() > MAX_HDNG_DIFF)
        return false;
    else return true;
}

bool ShipLanding::checkShipDroveOverLastWP()
{
    // Ship passes last wp formerly in front of ship TODO
    qCDebug(ShipLandingLog) << "lastWP:" << ship.coord.distanceTo(wp.at(3));
    if (ship.coord.distanceTo(wp.at(3)) < MIN_DIST_SHIP_WP)
        return false;
    else return true;
}

bool ShipLanding::checkMaxDistShipToPlane()
{
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DIST_PLANE_SHIP)
        return false;
    else return true;
}

bool ShipLanding::checkMaxDistWaterToPlane()
{
    if (water.coord.distanceTo(_vehicle->coordinate()) > MAX_DIST_PLANE_SHIP)
        return false;
    else return true;
}

bool ShipLanding::checkDistShipToHome()
{
    if (ship.coord.distanceTo(_vehicle->homePosition()) > MAX_DIST_PLANE_SHIP ||
        ship.coord.distanceTo(_vehicle->homePosition()) < MIN_DIST_PLANE_SHIP)
        return false;
    else return true;
}

//-Public slots-----------------------------------------------------------------------------------//

void ShipLanding::landingInit()
{
    qCDebug(ShipLandingLog) << "landingInit: Init Landing, ";
    timerObserve->start(TMR_INTVL.at(IDLE) * 1000);
    observeState();
    return;
}

void ShipLanding::landingStartRtS()
{
    qCDebug(ShipLandingLog) << "landingStart: Confirmed Landing";
    landReq = true;
    landWater = false;
    landCancel = false;
    landReset = false;
    observeState();
    return;
}

void ShipLanding::landingStartRtW()
{
    qCDebug(ShipLandingLog) << "landingStart: Confirmed Landing";
    landReq = true;
    landWater = true;
    landCancel = false;
    landReset = false;
    observeState();
    return;
}

void ShipLanding::landingCancel()
{
    qCDebug(ShipLandingLog) << "landingCancel: Canceled Landing";
    landReq = false;
    landWater = false;
    landCancel = true;
    landReset = false;
    observeState();
    return;
}

void ShipLanding::landingReset()
{
    qCDebug(ShipLandingLog) << "landingCancel: Canceled Landing";
    landReq = false;
    landWater = false;
    landCancel = false;
    landReset = true;
    observeState();
    return;
}

//-Private Slots----------------------------------------------------------------------------------//

void ShipLanding::sendHomePoint()
{
    qCDebug(ShipLandingLog) << "sendHomePoint: Send the home point.";

    if(_vehicle)
    {
        QGeoCoordinate newHome = calcPosRelativeToCoord(WPLIST_DIST.at(0), WPLIST_ALT.at(0), ship.coord);
        _vehicle->sendMavCommand(_vehicle->defaultComponentId()         ,
                                 MAV_CMD_DO_SET_HOME, true              ,
                                 0,                                             // Secified location
                                 0, 0, 0,                                       // Unused param 2-4
                                 static_cast<float>(newHome.latitude()) ,
                                 static_cast<float>(newHome.longitude()),
                                 static_cast<float>(newHome.altitude()));
    }

    return;
}

void ShipLanding::sendHomeWaterPoint()
{
    qCDebug(ShipLandingLog) << "sendHomeWaterPoint: Send the home water point.";

    if(_vehicle)
    {
        QGeoCoordinate newHome = calcPosRelativeToCoord(WPLIST_DIST.at(0), WPLIST_ALT.at(0), water.coord);
        _vehicle->sendMavCommand(_vehicle->defaultComponentId(),
                                 MAV_CMD_DO_SET_HOME, true,
                                 0,                                             // Secified location
                                 0, 0, 0,                                       // Unused param 2-4
                                 static_cast<float>(newHome.latitude()) ,
                                 static_cast<float>(newHome.longitude()),
                                 static_cast<float>(newHome.altitude()));
    }

    return;
}

void ShipLanding::sendHomeGoto()
{
    qCDebug(ShipLandingLog) << "sendHomeGoto: Send the home goto point.";
    _vehicle->guidedModeGotoLocation(_vehicle->homePosition());
    return;
}

void ShipLanding::sendFallbackGoAround()
{
    qCDebug(ShipLandingLog) << "sendFallbackGoAround: Send the fallback goto point.";
    _vehicle->guidedModeGotoLocation
                              (calcPosRelativeToCoord(GO_AROUND_DIST, GO_AROUND_ALT, ship.coord, GO_AROUND_HDG));
    return;
}

void ShipLanding::sendFallbackGoAroundWater()
{
    qCDebug(ShipLandingLog) << "sendFallbackGoAround: Send the fallback goto point.";
    _vehicle->guidedModeGotoLocation
                              (calcPosRelativeToCoord(GO_AROUND_DIST, GO_AROUND_ALT, water.coord, GO_AROUND_HDG));
    return;
}

void ShipLanding::sendGeofence()
{
    qCDebug(ShipLandingLog) << "sendGeofence: Build and send the landing geofence.";

    // If Geofence breached -> ReturnMode (GF_ACTION: 3, see Checklist Failure)
    QmlObjectListModel polygons, circles;
    QGCFencePolygon polygon = new QGCFencePolygon(true);
    QGeoCoordinate breach;
    polygon.appendVertex(calcPosRelativeToCoord(fabs(WPLIST_DIST.back()*GEOFENCE_MULTIPLY_DIST),
                                                WPLIST_ALT.at(2), ship.coord, GEOFENCE_ANGLE_NET));
    polygon.appendVertex(calcPosRelativeToCoord(GEOFENCE_DIST_CORRIDOR*GEOFENCE_MULTIPLY_DIST,
                                                WPLIST_ALT.at(1), ship.coord, GEOFENCE_ANGLE_SHIP));
    polygon.appendVertex(calcPosRelativeToCoord(GEOFENCE_DIST_CORRIDOR*GEOFENCE_MULTIPLY_DIST,
                                                WPLIST_ALT.at(1), ship.coord, -GEOFENCE_ANGLE_SHIP));
    polygon.appendVertex(calcPosRelativeToCoord(MAX_DIST_PLANE_SHIP*GEOFENCE_MULTIPLY_DIST,
                                             WPLIST_ALT.at(0), ship.coord, GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToCoord(MAX_DIST_PLANE_SHIP*GEOFENCE_MULTIPLY_DIST,
                                            WPLIST_ALT.at(0), ship.coord, -GEOFENCE_ANGLE_LOITER));
    polygons.insert(0, &polygon);
    _vehicle->geoFenceManager()->sendToVehicle(breach, polygons, circles);

    return;
}

void ShipLanding::sendGeofenceWater()
{
    qCDebug(ShipLandingLog) << "sendGeofenceWater: Build and send the landing geofence.";

    // If Geofence breached -> ReturnMode (GF_ACTION: 3, see Checklist Failure)
    QmlObjectListModel polygons, circles;
    QGCFencePolygon polygon = new QGCFencePolygon(true);
    QGeoCoordinate breach;
    polygon.appendVertex(calcPosRelativeToCoord(fabs(WPLIST_DIST.back()*GEOFENCE_MULTIPLY_DIST),
                                                WPLIST_ALT.at(2), water.coord, GEOFENCE_ANGLE_NET));
    polygon.appendVertex(calcPosRelativeToCoord(GEOFENCE_DIST_CORRIDOR*GEOFENCE_MULTIPLY_DIST,
                                                WPLIST_ALT.at(1), water.coord, GEOFENCE_ANGLE_SHIP));
    polygon.appendVertex(calcPosRelativeToCoord(GEOFENCE_DIST_CORRIDOR*GEOFENCE_MULTIPLY_DIST,
                                                WPLIST_ALT.at(1), water.coord, -GEOFENCE_ANGLE_SHIP));
    polygon.appendVertex(calcPosRelativeToCoord(MAX_DIST_PLANE_SHIP*GEOFENCE_MULTIPLY_DIST,
                                             WPLIST_ALT.at(0), water.coord, GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToCoord(MAX_DIST_PLANE_SHIP*GEOFENCE_MULTIPLY_DIST,
                                            WPLIST_ALT.at(0), water.coord, -GEOFENCE_ANGLE_LOITER));
    polygons.insert(0, &polygon);
    _vehicle->geoFenceManager()->sendToVehicle(breach, polygons, circles);

    return;
}

void ShipLanding::sendLandMission(int idx)
{
    qCDebug(ShipLandingLog) << "sendLandMission: Build and send the landing mission.";
    wp.clear();
    for (int i=0; i<WPLIST_DIST.count(); i++)
        wp.push_back(calcPosRelativeToCoord(WPLIST_DIST.at(i), WPLIST_ALT.at(i), ship.coord));
    qCDebug(ShipLandingLog) << "sendLandMission: WP-List=" << wp;

    // Convert waypoints into MissionItems
    QList<MissionItem*> landingItems;
    for (int i=0; i<wp.count(); i++)
    {
        landingItems.push_back
                (new MissionItem(i,                                     // sequence number
                                 MAV_CMD_NAV_WAYPOINT,                  // command
                                 MAV_FRAME_GLOBAL_INT,                  // frame
                                 0,                                     // param1 hold time
                                 WPLIST_ACCEPT_RAD.at(i),               // param2 acceptance radius
                                 0,                                     // param3 pass trough
                                 std::numeric_limits<int>::quiet_NaN(), // param4 yaw angle
                                 wp.at(i).latitude(),                   // param5 latitude
                                 wp.at(i).longitude(),                  // param6 longitude
                                 wp.at(i).altitude(),                   // param7 altitude
                                 true,                                  // autoContinue
                                 i == idx));                            // is current Item
    }
    qCDebug(ShipLandingLog) << "sendLandMission: landingItems=" << landingItems << ", idx=" << idx;
    _vehicle->missionManager()->writeMissionItems(landingItems);

    return;
}

void ShipLanding::sendLandMissionWater(int idx)
{
    qCDebug(ShipLandingLog) << "sendLandMissionWater: Build and send the landing mission.";
    wp.clear();
    for (int i=0; i<WPLIST_DIST.count(); i++)
        wp.push_back(calcPosRelativeToCoord(WPLIST_DIST.at(i), WPLIST_ALT.at(i), water.coord));
    qCDebug(ShipLandingLog) << "sendLandMissionWater: WP-List=" << wp;

    // Convert waypoints into MissionItems
    QList<MissionItem*> landingItems;
    for (int i=0; i<wp.count(); i++)
    {
        landingItems.push_back
                (new MissionItem(i,                                     // sequence number
                                 MAV_CMD_NAV_WAYPOINT,                  // command
                                 MAV_FRAME_GLOBAL_INT,                  // frame
                                 0,                                     // param1 hold time
                                 WPLIST_ACCEPT_RAD.at(i),               // param2 acceptance radius
                                 0,                                     // param3 pass trough
                                 std::numeric_limits<int>::quiet_NaN(), // param4 yaw angle
                                 wp.at(i).latitude(),                   // param5 latitude
                                 wp.at(i).longitude(),                  // param6 longitude
                                 wp.at(i).altitude(),                   // param7 altitude
                                 true,                                  // autoContinue
                                 i == idx));                            // is current Item
    }
    qCDebug(ShipLandingLog) << "sendLandMissionWater: landingItems=" << landingItems << ", idx=" << idx;
    _vehicle->missionManager()->writeMissionItems(landingItems);

    return;
}

void ShipLanding::observeState()
{
    static int idx = 1;

    if (!checkDistShipToHome())
    {
        if (state != IDLE)
            landWater ? sendHomeWaterPoint() : sendHomePoint();
        if (state == RETURN || state == LAND_REQ)
            sendHomeGoto();
    }

    switch (state)
    {
        case IDLE:
            qCDebug(ShipLandingLog) << "observeState: IDLE";
            if (_vehicle->missionManager()->currentIndex() > 0 || _vehicle->flying())
                state = MISSION;
            break;

        case MISSION:
            qCDebug(ShipLandingLog) << "observeState: MISSION";
            if (landReset)
                state = IDLE;
            else if (landReq)
                state = LAND_REQ;
            else if((_vehicle->missionManager()->currentIndex()
                    >= _vehicle->missionManager()->PlanManager::missionItems().count() - 1)
                    || (_vehicle->flightMode().compare(_vehicle->rtlFlightMode(), Qt::CaseInsensitive)
                    == 0))
                state = RETURN;
            break;

        case RETURN:
            qCDebug(ShipLandingLog) << "observeState: RETURN";
            if ((!landWater && !checkMaxDistShipToPlane()) || (landWater && !checkMaxDistWaterToPlane()))
                sendHomeGoto();
            if (landReset)
                {state = IDLE; landReset = false;}
            else if (landReq)
                state = LAND_REQ;
            break;

        case LAND_REQ:
            qCDebug(ShipLandingLog) << "observeState: LAND_REQ";
            if ((!landWater && !checkMaxDistShipToPlane()) || (landWater && !checkMaxDistWaterToPlane()))
                sendHomeGoto();
            if (landReset)
                {state = IDLE; landReset = false;}
            else if (landCancel)
                {state = RETURN; landCancel = false;}
            else if (checkPlaneNearHomePoint() && checkShipHeadingRate())
            {
                state = LAND_SEND;
                idx = 1;
            }
            break;

        case LAND_SEND:
            qCDebug(ShipLandingLog) << "observeState: LAND_SEND";
            landWater ? sendGeofenceWater() : sendGeofence();
            landWater ? sendLandMissionWater(idx) : sendLandMission(idx);
            dir_miss = ship.dir;
            qCDebug(ShipLandingLog) << "observeState: dir_miss=" << dir_miss;
            // Continue mission from waypoint idx, currently not available.
            // Mission always starts at index 1.
            _vehicle->startMission();
            // _vehicle->setCurrentMissionSequence(idx+1); no effect (timing?)
            state = LAND_APPROACH;
            break;

        case LAND_APPROACH:
            qCDebug(ShipLandingLog) << "observeState: LAND_APPROACH";
            if (landReset)
                {state = IDLE; landReset = false;}
            else if (landCancel)
                {state = RETURN; landCancel = false;}
            else if ((!landWater && !checkShipInLandingCorridor()) || (landWater && !checkWaterInLandingCorridor()) || !checkShipHeadingDifference())
            {
                double dist = ship.coord.distanceTo(_vehicle->coordinate());
                qCDebug(ShipLandingLog) << "observeState: dist=" << dist;
                if (dist > FALLBACK_DIST.at(0))
                    state = FALLBACK_RESTART_APPROACH;
                else if (dist > FALLBACK_DIST.at(1))
                    state = FALLBACK_RESTART_LOITER;
                else if (dist > FALLBACK_DIST.at(2))
                    state = FALLBACK_GO_AROUND;
                else if (dist > FALLBACK_DIST.at(3))
                    state = FALLBACK_PNR;
            }
            else if (!checkShipDroveOverLastWP())
                state = FALLBACK_RESTART_APPROACH;
            break;

        case FALLBACK_RESTART_APPROACH:
            qCDebug(ShipLandingLog) << "observeState: FALLBACK_RESTART_APPROACH";
            idx = _vehicle->missionManager()->currentIndex() + 1;
            state = LAND_SEND;
            break;

        case FALLBACK_RESTART_LOITER:
            qCDebug(ShipLandingLog) << "observeState: FALLBACK_RESTART_LOITER";
            sendHomeGoto();
            state = LAND_REQ;
            break;

        case FALLBACK_GO_AROUND:
            qCDebug(ShipLandingLog) << "observeState: FALLBACK_GO_AROUND";
            landWater ? sendFallbackGoAroundWater() : sendFallbackGoAround();
            state = RETURN;
            landReq = false;
            landWater = false;
            break;

        case FALLBACK_PNR:
            qCDebug(ShipLandingLog) << "observeState: FALLBACK_PNR";
            if (landReset)
                {state = IDLE; landReset = false;}
            else if (landCancel)
                {state = RETURN; landCancel = false;}
            break;
    }

    timerObserve->setInterval(TMR_INTVL.at(state) * 1000);
    qCDebug(ShipLandingLog) << "observeState: newState=" << state << "tmr=" << TMR_INTVL.at(state);
    return;
}

void ShipLanding::updatePosShip(QGeoPositionInfo update)
{
    // Safe the old GPS data to calculate heading
    // If NaN pos is HSA Etech
    QGeoCoordinate old_ship_pos = ship.coord;
    if (qIsNaN(ship.coord.longitude()) || qIsNaN(ship.coord.latitude()))
    {
        qCDebug(ShipLandingLog) << "update_posShip: Old ship pos is Nan.";
        old_ship_pos.setLongitude(HSA_ETECH_LAT);
        old_ship_pos.setLatitude(HSA_ETECH_LON);
    }

    // Transfer GPS data to struct
    ship.coord = update.coordinate();
    water.coord = calcPosRelativeToCoord(WATER_DIST, WATER_ALT, ship.coord, WATER_ANGLE);
    if (update.hasAttribute(update.Direction))
        ship.dir = update.attribute(update.Direction);
    else {

       /*
        * Calculate heading. TODO: Replace by values from GPS sensor.
        * Idea: Start heading with 0 degrees.
        * Afterwards, calculate a heading from old and new GPS.
        * Add this heading to the new one, but weighed in with a factor.
        * Please note that this won't work at 180 degrees longitude, but we are at mediteranian sea and
        * this is supposed to be a workaround anyway, so I won't bother building this to work at 180
        * degrees.
        */
        double alpha;   // Alpha shows the angle in a normal coordinate system.
        double new_dir; // New direction.

        // Check for  0 degree and 180 degree (see definition of atan!)
        if (ship.coord.longitude() == old_ship_pos.longitude())
        {
            // Heading didn't change
            if (ship.coord.latitude() == old_ship_pos.latitude())
                new_dir = ship.dir;
            else if (ship.coord.latitude() > old_ship_pos.latitude())
                new_dir = NORTH;    // 0 degrees
            else
                new_dir = SOUTH;    // 180 degrees
        }
        else
        {
            alpha = atan((ship.coord.latitude() - old_ship_pos.latitude())/
                         (ship.coord.longitude() - old_ship_pos.longitude()));

            // alpha > 0 means 1. quadrant, alpha < 0 means 2. quadrant
            if (ship.coord.longitude() > old_ship_pos.longitude())
                new_dir = EAST - alpha;
            // alpha > 0 means 3. quadrant, alpha < 0 means 4. quadrant
            else
                new_dir = WEST - alpha;
        }

        ship.dir = ship.dir + (HDNG_WEIGHT * (new_dir - ship.dir));

        // Put new direction in history queue
        _Heading new_entry = {ship.dir, update.timestamp()};
        ship.hdng_his.push_back(new_entry);
        if (ship.hdng_his.size() > HDNG_HIS_SIZE)
            ship.hdng_his.pop_front();

    }
    return;
}

//-EOF--------------------------------------------------------------------------------------------//
