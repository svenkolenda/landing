/*-Includes-------------------------------------------------------------------*/

#include "ShipLanding.h"

ShipLanding* ShipLanding::_instance = new ShipLanding();

/*-Local defines--------------------------------------------------------------*/

#define MAX_DISTANCE_TO_SHIP                 200   // meter
#define LOITER_DISTANCE_TO_SHIP              30    // meter
#define LOITER_UPDATE                        10    // second

/*-Public functions-----------------------------------------------------------*/

ShipLanding* ShipLanding::getInstance()
{
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

void ShipLanding::prepareToLoiter()
/** Entry point for the landing procedure.
 * Starts the timerLoiter. Aks for the user's okay to init the landing. */
{
    start_timerLoiter();
    initDialog();				// ToDo: Check function without closing MsgBox
}

/*-Private functions----------------------------------------------------------*/

ShipLanding::ShipLanding(QObject *parent) : QObject(parent), _vehicle(NULL)
/** Connects slots and signals, configures the timerLoiter. */
{
    //Connect our buttons to corresponding functions.
    connect(this, &ShipLanding::initDialog_yes, this, &ShipLanding::land);
    connect(this, &ShipLanding::cancelDialog_yes, this, &ShipLanding::prepareToLoiter);

    // TODO: Connect GQCPositionManager to USB_GPS
    connect(qgc->toolbox()->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated,
            this, &ShipLanding::update_posPlane);
    // TODO: Position plane
    //connect(qgc->toolbox()->PositionManager(), &PlanePositionManager::positionInfoUpdated, this, &ShipLanding::update_posPlane);

    // Timers
    timerLoiter->setSingleShot(false);
    connect(timerLoiter, &QTimer::timeout, this, &ShipLanding::loiterShip);

    // Vehicl
    if(qgcApp()->toolbox()->multiVehicleManager()->activeVehicle()) {
        _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    }

    // TODO: Connect prepare to land to end of mission
}

ShipLanding::~ShipLanding()
{
}

void ShipLanding::calculateDistance(__GPS plane, __GPS ship)
/** Calculate the distance between plane and ship. Allow for the earth's curvature. */
{
    distance = 0;
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

bool ShipLanding::initDialog()
/** Message Box to init the landing. */
{
    // TODO: QGC Slider
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Land", "Quit?", QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
        emit initDialog_yes();
        return true;
    }
    else
    {
        emit initDialog_no();
        return false;
    }
}

bool ShipLanding::cancelDialog()
/** Message Box to cancel the Landing. */
{
    // TODO: QGC Slider
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Cancel Land", "Quit?", QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
        emit cancelDialog_yes();
        return true;
    }
    else
    {
        emit cancelDialog_yes();
        return false;
    }
}

/*-Private Slots--------------------------------------------------------------*/

void ShipLanding::loiterShip()
/** Build and send the loiter message to the plane. */
{
    calculateDistance(plane, ship);

    if (distance > MAX_DISTANCE_TO_SHIP)
    {
        /*
      * Send the plane 100m behind the ship.
      */
    }
}

void ShipLanding::land()
/** Called after user starts the landing. Stops the timerLoiter.
 * Build and send landing mission to plane. Observe the ship movement.
 * Provides the cancel option for the user. */
{
    stop_timerLoiter();							// stop GPS from initiating Loiter

    cancelDialog();							// TODO: parallel Execution ??

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
