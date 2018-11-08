/*-Includes-------------------------------------------------------------------*/

#include "tp_landing.h"

/*-Local defines--------------------------------------------------------------*/

#define MAX_DISTANCE_TO_SHIP                 200   // meter
#define LOITER_DISTANCE_TO_SHIP              30    // meter
#define LOITER_UPDATE                        10    // second

/*-Public functions-----------------------------------------------------------*/

TP_Landing::TP_Landing(QObject *parent) : QObject(parent)
{
   //Connect our buttons to corresponding functions.
   connect(this, &TP_Landing::initDialog_yes, this, &TP_Landing::land);
   connect(this, &TP_Landing::cancelDialog_yes, this, &TP_Landing::prepareToLoiter);
   
   // TODO: Connect GQCPositionManager to USB_GPS
   connect(qgc->_toolbox->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated,
												this, &TP_Landing::update_posPlane);
   // TODO: Position plane
   //connect(qgc->_toolbox->qgcPositionManager(), &QGCPositionManager::positionInfoUpdated,
												this, &TP_Landing::update_posPlane);

   // Timers
   timerLoiter->setSingleShot(false);
   connect(timerLoiter, &QTimer::timeout, this, &TP_Landing::loiterShip);
   
   // TODO: Connect prepare to land to end of mission
}

TP_Landing::~TP_Landing()
{

}

/*-Public slots---------------------------------------------------------------*/

void TP_Landing::prepareToLoiter()
{
   start_timerLoiter();
   land_initDialog();				// ToDo: Check function without closing MsgBox
}

/*-Private functions----------------------------------------------------------*/

void TP_Landing::calculateDistance(__GPS plane, __GPS ship)
{
   distance = 0;
}

void TP_Landing::start_timerLoiter()
/** Start the timer for loiter update. */
{
    timerLoiter->start(LOITER_UPDATE * 1000);
}

void TP_Landing::stop_timerLoiter()
/** Stop the timer for loiter update. */
{
    timerLoiter->stop();
}

bool TP_Landing::initDialog()
/** Message Box to init the landing. */
{
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

bool TP_Landing::cancelDialog()
/** Message Box to cancel the Landing. */
{
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

void TP_Landing::loiterShip()
{
   calculateDistance(plane, ship);

   if (distance > MAX_DISTANCE_TO_SHIP)
   {
     /*
      * Send the plane 100m behind the ship.
      */
   }
}

void TP_Landing::land()
{
   stop_timerLoiter();							// stop GPS from initiating Loiter
   
   land_cancelDialog();							// TODO: parallel Execution ??

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

void TP_Landing::update_posPlane()
{
	// Transfer GPS data to struct
	// TODO: QGCPositionManager => __GPS
}

void TP_Landing::update_posShip()
{
	// Transfer GPs data to struct
	// TODO: QGCPositionManager => __GPS
}
