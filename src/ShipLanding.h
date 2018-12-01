#ifndef ShipLANDING_H
#define ShipLANDING_H

/*-Includes-------------------------------------------------------------------*/

#include <stdint.h>
#include <math.h>

#include <QObject>
#include <QTimer>
#include <QGeoCoordinate>

#include "QGCApplication.h"
#include "QGCLoggingCategory.h"
#include "PositionManager.h"
#include "Vehicle.h"
#include "MissionManager.h"
#include "QmlObjectListModel.h"
#include "QGCFencePolygon.h"

/*-Local defines--------------------------------------------------------------*/

Q_DECLARE_LOGGING_CATEGORY(ShipLandingLog)

/**
 * @brief Struct to save the coordinates and the direction of plane/ship
 *
 */
struct __GPS
{
    QGeoCoordinate coord;
    double dir = 0;
};

/*-ShipLanding-----------------------------------------------------------------*/
// Timer interval in seconds
const int TMR_INTVL_LOITER = 120;  //!< Timer intervall to check loiter
const int TMR_INTVL_OBS = 60;      //!< Timer intervall to observe landing

/*!
 * \brief The ShipLanding class
 */
class ShipLanding : public QObject
{
    Q_OBJECT

private:    // attributes
    static ShipLanding* _instance;              //!< Singleton instance of ShipLanding
    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    QTimer* timerLoiter = new QTimer(this);     //!< Timer to update loiter
    QTimer* timerObserve = new QTimer(this);    //!< Timer to observe landing
    __GPS ship;                                 //!< Information struct of ship

public:     // functions
    /*!
     * \brief Returns the instance for the QML-Integration.
     * \param engine QQmlEnging to connect Q_INVOKABLE to QML
     * \param scriptEngine QJEnginge to parse to JSON
     * \return _instance of ShipLanding
     */
    static QObject* qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine);
    /*!
     * \brief Delete the instance and set _instance to nullptr.
     */
    static void release();

private:    // functions
    /*!
     * \brief Connect landing and cancel, connect PositionManger ship and plane, configure the timerLoiter.
     * \param parent QObject parent from the ShipLanding object
     */
    ShipLanding(QObject *parent = nullptr);
    /*!
     * \brief Private Destructor.
     */
    ~ShipLanding();

    // Start and stop-functions for the timer
    void start_timerLoiter() {timerLoiter->start(TMR_INTVL_LOITER * 1000);}
    void stop_timerLoiter() {timerLoiter->stop();}
    void start_timerObserve() {timerObserve->start(TMR_INTVL_OBS * 1000);}
    void stop_timerObserve() {timerLoiter->stop();}

    /*!
     * \brief Calculate the position distance away from ship resting upon the heading.
     * \param distance planned distance behind ship
     * \param alltitude planned absolute alltitude
     * \return
     */
    QGeoCoordinate calcPosRelativeToShip(double, unsigned int, double);

public slots:
    /*!
     * \brief Reaction to init the landing. Called by UI.
     */
    Q_INVOKABLE void landingStart();
    /*!
     * \brief Reaction to cancel the Landing. Called by UI.
     */
    Q_INVOKABLE void landingCancel();

private slots:
    /*!
     * \brief Build and send the loiter message to the plane.
     */
    void loiterSend();
    /*!
     * \brief Initiate the landing process.
     * Called after user starts the landing. Stops the timerLoiter.
     * Build and send landing mission to plane. Observe the ship movement.
     * Provides the cancel option for the user.
     */
    void landSend();
    /*!
     * \brief Observe ship heading, ship position relative to wp2 (in front of ship). Send mission if needed.
     */
    void landObserve();     //APF

    /*!
     * \brief Called when the plane moved. Update the saved location and heading.
     */
    void update_posShip(QGeoPositionInfo update);

signals:
    void confirmLandingStart();      //!< Signal to start landing process
    void confirmLandingCancel();     //!< Signal to cancel landing proces and start loiter
};

#endif // ShipLANDING_H
