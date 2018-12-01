/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

QGC_LOGGING_CATEGORY(ShipLandingLog, "ShipLandingLog")

/*-Local defines--------------------------------------------------------------*/
// Distance in meter
const double MAX_DISTANCE = 1000;   //!< Maximum distance plane to ship
const double NET_DISTANCE = 3/2;    //!< Distance ship position to end of net

// WP-List 0: Loiter, 1: DownToAltitude, 2: WP behind ship, 3: WP in front of ship
const QList<double> WPLIST_DIST({750, 500, 250, -250});         //!< Distance plane to ship
const QList<unsigned int> WPLIST_ALT({50, 15, 5, 5});           //!< Altitude (absolute)
const QList<unsigned int> WPLIST_ACCEPT_RAD({15, 10, 5, 1});    //!< Acceptance radius for the waypoint

// Calculation parameters for distance vs coordinates
const unsigned int GAP_LATITUDE = 111300;   //!< gap between circles of latitude
const double DEGREE_TO_RAD = M_PI / 180;    //!< conversion degree to radian

// Cartesian Coordinate System: x-axis with 0 degree going counterclockwise
// Heading System: NORTH (y-axis) with 0 degress going clockwise
const double HEADING_WEIGHT = 0.1;                          //!< heading weight
const int NORTH = 0, EAST = 90, SOUTH = 180, WEST = 270;    //!< heading angles

// Default position
const double HSA_ETECH_LAT = 48.354772;
const double HSA_ETECH_LON = 10.904693;

/*-Public functions-----------------------------------------------------------*/

QObject* ShipLanding::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine);
    Q_UNUSED(scriptEngine);

    qCDebug(ShipLandingLog) << "Start of ShipLanding";

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

/*-Private functions----------------------------------------------------------*/

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
    start_timerLoiter();
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

    qCDebug(ShipLandingLog) << "pos: " << pos << " | dir: " << ship.dir+dDir
                 << " | dx: " << dx << " | dy: " << dy
                 << " | distanceTo ship: " << ship.coord.distanceTo(pos);
    return pos;
}

/*-Public slots---------------------------------------------------------------*/

void ShipLanding::landingStart()
{
    qCDebug(ShipLandingLog) << "Confirmed Landing";
    start_timerObserve();               // start timer to observe landing
    landObserve();
}

void ShipLanding::landingCancel()
{
    qCDebug(ShipLandingLog) << "Confirmed Cancel";
    stop_timerObserve();                // stop timer to observe landing
}

/*-Private Slots--------------------------------------------------------------*/

void ShipLanding::loiterSend()
{
    qCDebug(ShipLandingLog) << "Send the loiter message.";

    // Check distance plane - ship
    if (ship.coord.distanceTo(_vehicle->coordinate()) > MAX_DISTANCE)
    {
        // Send the plane to the loiter waypoint behind the ship
        if(_vehicle)
        {
            qCDebug(ShipLandingLog) << "INDEX" << _vehicle->missionManager()->currentIndex()
                     << "LASTINDEX" << _vehicle->missionManager()->lastCurrentIndex();
            if(_vehicle->missionManager()->currentIndex() == _vehicle->missionManager()->lastCurrentIndex())
            {
                qCDebug(ShipLandingLog) << "GOTO";
                _vehicle->guidedModeGotoLocation(calcPosRelativeToShip(WPLIST_DIST.at(0), WPLIST_ALT.at(0)));
            }
            else if(_vehicle->missionManager()->currentIndex() > 0)
            {
                qCDebug(ShipLandingLog) << "SETHOME";
                QGeoCoordinate newHome = calcPosRelativeToShip(WPLIST_DIST.at(0), WPLIST_ALT.at(0));
                _vehicle->sendMavCommand(_vehicle->defaultComponentId(), MAV_CMD_DO_SET_HOME, true,
                                         0, 0, 0, 0,        // unused param 1-4
                                         static_cast<float>(newHome.latitude()),
                                         static_cast<float>(newHome.longitude()),
                                         static_cast<float>(newHome.altitude()));
            }
        }
     }
}

void ShipLanding::landSend()
{
    qDebug() << "Build and send the landing mission";
    QList<QGeoCoordinate> wp;
    for (int i=1; i<WPLIST_DIST.count(); i++)
    {
        wp.push_back(calcPosRelativeToShip(WPLIST_DIST.at(i), WPLIST_ALT.at(i)));
    }
    qCDebug(ShipLandingLog) << wp;

    // Convert Waypoints into MissionItems
    QList<MissionItem*> landingItems;
    for (int i=1; i<wp.count(); i++)
    {
        landingItems.push_back
            (new MissionItem(i+1,     // sequence number
             MAV_CMD_NAV_WAYPOINT,    // command
             MAV_FRAME_GLOBAL,        // frame
             0,          // param1 hold time
             WPLIST_ACCEPT_RAD.at(i), // param2 acceptance radius
             0,                       // param3 pass trough
             std::numeric_limits<int>::quiet_NaN(),   // param4 yaw angle
             wp.at(i).latitude(),     // param5 latitude
             wp.at(i).longitude(),    // param6 longitude
             wp.at(i).altitude(),     // param7 altitude
             true,                    // autoContinue
             i==1));                  // is current Item
    }
    qCDebug(ShipLandingLog) << landingItems;
    _vehicle->missionManager()->writeMissionItems(landingItems);

    // Bei Uebertritt Geofence -> ReturnMode (Parameter GF_ACTION auf 3 setzen PX4MockLink.params, V1.4.OfflineEditing.params)
    QmlObjectListModel polygons, circles;
    QGCFencePolygon polygon = new QGCFencePolygon(true);
    QGeoCoordinate breach = wp.back();
    polygon.appendVertex(calcPosRelativeToShip(NET_DISTANCE, WPLIST_ALT.at(1), 180));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0), 15));
    polygon.appendVertex(calcPosRelativeToShip(MAX_DISTANCE, WPLIST_ALT.at(0), -15));
    polygon.appendVertex(calcPosRelativeToShip(-NET_DISTANCE, WPLIST_ALT.at(1), -180));
    polygons.insert(0, &polygon);
    //_vehicle->geoFenceManager()->sendToVehicle(breach, polygons, circles);

    //_vehicle->startMission();
}

void ShipLanding::landObserve()
{
    // TODO: Watch ship heading rate of change

    // TODO: Watch ship position relative to wp2 (in front of ship)

    // TODO: Check for current waypoint

    // TODO: if ...
    landSend();
}

void ShipLanding::update_posShip(QGeoPositionInfo update)
{
   // Safe the old GPS data to calculate heading. If NaN pos is HSA Etech.
    QGeoCoordinate old_ship_pos = ship.coord;
    if (qIsNaN(ship.coord.longitude()) || qIsNaN(ship.coord.latitude()))
    {
        qCDebug(ShipLandingLog) << "Old ship pos is Nan.";
        old_ship_pos.setLongitude(HSA_ETECH_LAT);
        old_ship_pos.setLatitude(HSA_ETECH_LON);
    }

    // Transfer GPs data to struct
     ship.coord = update.coordinate();
     qCDebug(ShipLandingLog) << "Ship coord: " << ship.coord;

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
    qCDebug(ShipLandingLog) << "Ship direction: " << ship.dir;
}
