/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = nullptr;

/*-Local defines--------------------------------------------------------------*/

#define MAX_DISTANCE_TO_SHIP                 300    // meter
#define LOITER_DISTANCE_TO_SHIP              100    // meter
#define LOITER_UPDATE                        10     // second
#define LOITER_ALTITUDE                      50     // meter

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
    connect(this, &ShipLanding::confirmCancel, this, &ShipLanding::prepareToLoiter);

    // TODO: Connect GQCPositionManager to USB_GPS
    connect(qgcApp()->toolbox()->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated, this, &ShipLanding::update_posPlane);
    // TODO: Position plane
    //connect(qgcApp()->toolbox()->PositionManager(), &PlanePositionManager::positionInfoUpdated, this, &ShipLanding::update_posPlane);

    // Timers
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterShip);

    // Vehicle
    if(qgcApp()->toolbox()->multiVehicleManager()->activeVehicle()) {
        _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    }

    // TODO: Connect prepare to land to end of mission
}

ShipLanding::~ShipLanding()
{
}

QGeoCoordinate ShipLanding::calcLoiterPos()
/** Calculate the position LOITER_DISTANCE_TO_SHIP away from Ship resting upon the heading. */
{
    /**TESTING START
    ship.dir =225;
    ship.coord.setAltitude(500);
    ship.coord.setLatitude(47.4065160);
    ship.coord.setLongitude(8.5425730);
    TESTING END */

    QGeoCoordinate pos;
    double longitude = 0, latitude = 0;

    longitude = sin(ship.dir * M_PI / 180) * LOITER_DISTANCE_TO_SHIP;
    latitude = cos(ship.dir * M_PI / 180) * LOITER_DISTANCE_TO_SHIP;

    pos.setLatitude(ship.coord.latitude() - (latitude/111300));
    pos.setLongitude(ship.coord.longitude() - (longitude/(111300*cos(ship.coord.latitude()*(M_PI / 180)))));
    pos.setAltitude(LOITER_ALTITUDE);
    //Testing: qDebug() << pos.longitude() << "||" << pos.latitude() << "||" << longitude << "||" << latitude << "||" <<ship.coord.distanceTo(pos);
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
 * Starts the timerLoiter. Aks for the user's okay to init the landing. */
{
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
    stop_timerLoiter();							// stop GPS from initiating Loiter

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
/** Called when the ship moved. Update the saved location and heading. */
{
    // Transfer GPS data to struct
    // TODO: QGCPositionManager => __GPS
}

void ShipLanding::update_posShip()
/** Called when the plane moved. Update the saved location and heading. */
{
    // Transfer GPs data to struct
    // TODO: QGCPositionManager => __GPS
}
