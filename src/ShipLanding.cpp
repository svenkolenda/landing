//-Includes------------------------------------------------------------------//

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

QGC_LOGGING_CATEGORY(ShipLandingLog, "ShipLandingLog")

//-Local defines-------------------------------------------------------------//
// Distance in meter
const double MAX_DISTANCE = 500;       //!< Maximum distance plane to ship
const double NET_DISTANCE = 3/2;        //!< Distance ship position to end of net

// WP-List 0: Loiter, 1: DownToAlt, 2: WP behind ship, 3: WP in front of ship
const QList<double> WPLIST_DIST({350, 250, 100, -100});         //!< Distance plane to ship
const QList<unsigned int> WPLIST_ALT({50, 15, 5, 5});           //!< Altitude (relative)
const QList<unsigned int> WPLIST_ACCEPT_RAD({15, 10, 5, 1});    //!< Acceptance radius for the waypoint
const double GEOFENCE_ANGLE_NET = 180;      //!< Geofence angle next to net
const double GEOFENCE_ANGLE_LOITER = 25;  //!< Geofence angle behind ship
const double FALLBACK_DIST = -100;      //!< Distance plane to ship
const unsigned int FALLBACK_ALT = 200;  //!< Altitude (relative)
const int FALLBACK_HDG = 45;            //!< Heading relative to ship

// Calculation parameters for distance vs coordinates
const unsigned int GAP_LATITUDE = 111300;   //!< gap between circles of latitude
const double DEGREE_TO_RAD = M_PI / 180;    //!< conversion degree to radian

// Cartesian Coordinate System: x-axis with 0 degree going counterclockwise
// Heading System: NORTH (y-axis) with 0 degress going clockwise
const double HEADING_WEIGHT = 0.1;                          //!< heading weight
const int HEADING_HIS_SIZE = 10;    //!< Heading history size
const int NORTH = 0, EAST = 90, SOUTH = 180, WEST = 270;    //!< heading angles
const int MAX_HDNG_DIFF = 10;       //!< Maximum acceptable heading difference
const int MAX_HDNG_RATE = 2;        //!< Maximum acceptable heading change in degrees per second

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

    // Initialize the timerLoiter
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterSend);
    // Initialize the timerObserve
    timerObserve->setSingleShot(false);
    connect(timerObserve, &QTimer::timeout, this, &ShipLanding::landObserve);

    // TODO: Connect prepare to land to end of mission

    // Set landObserve no landing
    landing = false;
}

ShipLanding::~ShipLanding() {}

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

void ShipLanding::sendBehindShip()
{
    qCDebug(ShipLandingLog) << "sendBehindShip: Send the loiter message.";
    if(_vehicle)
    {
        qCDebug(ShipLandingLog) << "MISSION_ITEM_COUNT:"
                                << _vehicle->missionManager()->PlanManager::missionItems().count()
                                << "CURRENT_INDX:"
                                << _vehicle->missionManager()->currentIndex();
        if(_vehicle->missionManager()->currentIndex() >=
           _vehicle->missionManager()->PlanManager::missionItems().count() - 1)
        {
            qCDebug(ShipLandingLog) << "GOTO";
            _vehicle->guidedModeGotoLocation(calcPosRelativeToShip(
                                          WPLIST_DIST.at(0), WPLIST_ALT.at(0)));
        }
        else if(_vehicle->missionManager()->currentIndex() > 0)
        {
            qCDebug(ShipLandingLog) << "SETHOME";
            QGeoCoordinate newHome = calcPosRelativeToShip(
                                           WPLIST_DIST.at(0), WPLIST_ALT.at(0));
            _vehicle->sendMavCommand(_vehicle->defaultComponentId(),
                                    MAV_CMD_DO_SET_HOME, true,
                                    0,              // specified location
                                    0, 0, 0,        // unused param 2-4
                                    static_cast<float>(newHome.latitude()),
                                    static_cast<float>(newHome.longitude()),
                                    static_cast<float>(newHome.altitude()));
        }
    }
}

QGeoCoordinate ShipLanding::calcFailsafe()
{
    QGeoCoordinate failsafe;
    // TODO: APF
    return failsafe;
}

int ShipLanding::calcHeadingRate()
{
    int heading_rate = 0;
    double xa, ya, xsum = 0, ysum = 0; //x average, y average, x sum, y sum
    double up = 0, down = 0;

    // calculate averages first
    for (unsigned int i = 0; i < ship.dir_his.size(); i++)
    {
        xsum += i;
        ysum += ship.dir_his.at(i);
    }
    xa = xsum / ship.dir_his.size();
    ya = ysum / ship.dir_his.size();

    // now, calculate what's on the fracture
    for (unsigned int i = 0; i < ship.dir_his.size(); i++)
    {
        up += (i - xa) * (ship.dir_his.at(i) - ya);
        down += (i - xa) * (i - xa);
    }

    if (down != 0)
        heading_rate = up / down;

    // TODO: Multiply with update interval of updateposShip

    return heading_rate;
}

int ShipLanding::calcHeadingDiff()
{
    int heading_difference = 0;
    //(fabs(ship.dir - dir_miss)); //TODO Berechnung aus akt. ship.dir und dir_miss
    return heading_difference;
}



//-Public slots--------------------------------------------------------------//

void ShipLanding::landingStart()
{
    qCDebug(ShipLandingLog) << "landingStart: Confirmed Landing";
    start_timerObserve();               // start timer to observe landing
    landObserve();
}

void ShipLanding::landingCancel()
{
    qCDebug(ShipLandingLog) << "landingCancel: Confirmed Cancel";
    stop_timerObserve();                // stop timer to observe landing
    start_timerLoiter();
}



//-Private Slots-------------------------------------------------------------//

void ShipLanding::loiterSend()
{
    qCDebug(ShipLandingLog) << "loiterSend: Check for loiter message.";
    // Check distance plane - ship
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DISTANCE)
    {
        sendBehindShip();
    }
}

void ShipLanding::landSend()
{
    //calc heading difference through direction when Mission uploaded
    dir_miss = ship.dir;

    qCDebug(ShipLandingLog) << "landSend: Build and send the landing mission";
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

    // Bei Geofence -> ReturnMode (GF_ACTION: 3, s.a. Checklist Failure)
    QmlObjectListModel polygons, circles;
    QGCFencePolygon polygon = new QGCFencePolygon(true);
    QGeoCoordinate breach = wp.back();
    polygon.appendVertex(calcPosRelativeToShip(NET_DISTANCE, WPLIST_ALT.at(1),
                                               GEOFENCE_ANGLE_NET));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0),
                                               GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0),
                                               -GEOFENCE_ANGLE_LOITER));
    polygon.appendVertex(calcPosRelativeToShip(-NET_DISTANCE, WPLIST_ALT.at(1),
                                               -GEOFENCE_ANGLE_NET));
    polygons.insert(0, &polygon);
    //_vehicle->geoFenceManager()->sendToVehicle(breach, polygons, circles);

    //_vehicle->startMission();
}

void ShipLanding::landObserve()
{
    // Check vehicle.
    if (!_vehicle)
    {
        qCDebug(ShipLandingLog) << "landObserve: Connection to vehicle lost.";
        return;
    }

    // Check plane in landing procedure
    if (!landing)
    {
        // Check if plane has minimum distance to ship and is behind ship.
        // TODO: Ask SKO if we can seperate the calculation from calcPosRelat...
        bool plane_in_area = true;
        if (!plane_in_area)
        {
            qCDebug(ShipLandingLog) << "landObserve: Plane not in landing area.";
            landing = false;
            sendBehindShip();
        }
        // Calculate the heading rate of ship_position at the moment
        else if (calcHeadingRate() > MAX_HDNG_RATE)
        {
            qCDebug(ShipLandingLog) << "landObserve: Too much heading change!";
            landing = false;
            sendBehindShip();
        }
        // Start the landing
        else
        {
            qCDebug(ShipLandingLog) << "landObserve: Initiate landing.";
            landing = true; //We are now in the process of landing.
            landSend();
        }

        return;
    }

    // Check heading diff while plane is in landing process
    if (calcHeadingDiff() > MAX_HDNG_DIFF)
    {
        qCDebug(ShipLandingLog) << "landObserve: Landing impossible. Fallback.";

        if ((ship.coord - plane.coord) < PONR)
        {
            // TODO: Fallback
        }
        // HDG - 180 because the function assumes 0 to be behind ship
        _vehicle->guidedModeGotoLocation
            (calcPosRelativeToShip
                             (FALLBACK_DIST, FALLBACK_ALT, FALLBACK_HDG - 180));
        sendBehindShip();
        landing = false;
    }
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
    * Calculate heading.
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

    // Put new direction in direction history queue
    ship.dir_his.push_back(ship.dir);
    if (ship.dir_his.size() > HEADING_HIS_SIZE)
        ship.dir_his.pop_front();
}
