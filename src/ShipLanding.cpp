/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

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
    connect(this, &ShipLanding::confirmCancel, this, &ShipLanding::prepareToLoiter);

    // TODO: Connect GQCPositionManager to USB_GPS
    connect(qgcApp()->toolbox()->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated, this, &ShipLanding::update_posPlane);
    // TODO: Position plane
    //connect(qgcApp()->toolbox()->PositionManager(), &PlanePositionManager::positionInfoUpdated, this, &ShipLanding::update_posPlane);

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
#if TEST == 1
    ship.dir = 225;
    ship.coord.setAltitude(500);
    ship.coord.setLatitude(47.4065160);
    ship.coord.setLongitude(8.5425730);
#endif

    QGeoCoordinate pos;
    double dx = 0, dy = 0;
    const unsigned int gapLatitude = 111300;
    const double degreeToRad = M_PI / 180;

    dx = sin(ship.dir * degreeToRad) * distance;
    dy = cos(ship.dir * degreeToRad) * distance;

    pos.setLatitude(ship.coord.latitude() - dy / gapLatitude);
    pos.setLongitude(ship.coord.longitude() - dx / gapLatitude * cos(ship.coord.latitude() * degreeToRad));
    pos.setAltitude(alltitude);
#if TEST == 1
    qDebug() << "Longitude: " << pos.longitude() << " | Latitude: " << pos.latitude()
             << " | dx: " << longitude << " | dy: " << latitude
             << " | distanceTo ship: " <<ship.coord.distanceTo(pos);
#endif
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
    // Transfer GPs data to struct
    // TODO: QGCPositionManager => __GPS
}
