#ifndef ShipLANDING_H
#define ShipLANDING_H

/*-Includes-------------------------------------------------------------------*/

#include <stdint.h>

#include <QObject>
#include <QMessageBox>
#include <QTimer>

#include "QGCApplication.h"
#include "PositionManager.h"

#include <QGeoCoordinate>
#include "Vehicle.h"

#include <math.h>

/*-Local defines--------------------------------------------------------------*/

#ifndef TEST
#define TEST 0
#endif

const unsigned int MAX_DISTANCE = 200;      //!< Maximum distance plane to ship
const unsigned int LOITER_DISTANCE = 150;   //!< Distance plane to ship for loiter point
const unsigned int LOITER_ALTITUDE = 50;    //!< Altitude (absolute) of the loiter point
const unsigned int LOITER_UPDATE = 10;      //!< Timer intervall to check loiter
const unsigned int WP2_DISTANCE = 100;      //!< Distance plane to ship for wp2
const unsigned int WP2_ALTITUDE = 25;       //!< Altitude (absolute) for wp2
const unsigned int WP3_DISTANCE = 50;       //!< Distance plane to ship for wp3
const unsigned int WP3_ALTITUDE = 5;        //!< Altitude (absolute) for wp3
const unsigned int WP4_DISTANCE = 0;        //!< Distance plane to ship for wp4
const unsigned int WP4_ALTITUDE = 5;        //!< Altitude (absolute) for wp4

/**
 * @brief Struct to save the coordinates and the direction of plane/ship
 *
 */
struct __GPS
{
    QGeoCoordinate coord;
    double dir;
};

/*-ShipLanding-----------------------------------------------------------------*/

/*!
 * \brief The ShipLanding class
 */
class ShipLanding : public QObject
{
    Q_OBJECT

public:
    /*!
     * \brief Returns the instance for the QML-Integration.
     * \param engine QQmlEnging to connect Q_INVOKABLE to QML
     * \param scriptEngine QJEnginge to parse to JSON
     * \return
     */
    static QObject* qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine);
    /*!
     * \brief Delete the instance and set _instance to nullptr.
     */
    static void release();

public slots:
    /*!
     * \brief Reaction to init the landing.
     */
    Q_INVOKABLE void initDialog();
    /*!
     * \brief Reaction to cancel the Landing.
     */
    Q_INVOKABLE void cancelDialog();

private:
    static ShipLanding* _instance;              //!< Singleton instance of ShipLanding
    QTimer* timerLoiter = new QTimer(this);     //!< Timer to update loiter
    __GPS plane;                                //!< Information struct of plane
    __GPS ship;                                 //!< Information struct of ship

    /*!
     * \brief Connect landing and cancel, connect PositionManger ship and plane, configure the timerLoiter.
     * \param parent QObject parent from the ShipLanding object
     */
    explicit ShipLanding(QObject *parent = nullptr);
    /*!
     * \brief Private Destructor.
     */
    ~ShipLanding();

    /*!
     * \brief Calculate the position distance away from ship resting upon the heading.
     * \param distance planned distance behind ship
     * \param alltitude planned absolute alltitude
     * \return
     */
    QGeoCoordinate calcPosRelativeToShip(unsigned, unsigned);

    /*!
     * \brief Start the timer for loiter update.
     */
    void start_timerLoiter();
    /*!
     * \brief Stop the timer for loiter update.
     */
    void stop_timerLoiter();

private slots:
    /*!
     * \brief Entry point for the landing procedure. Starts the timerLoiter.
     */
    void prepareToLoiter();
    /*!
     * \brief Build and send the loiter message to the plane.
     */
    void loiterShip();
    /*!
     * \brief Initiate the landing process.
     * Called after user starts the landing. Stops the timerLoiter.
     * Build and send landing mission to plane. Observe the ship movement.
     * Provides the cancel option for the user.
     */
    void land();

    /*!
     * \brief Called when the ship moved. Update the saved location and heading.
     */
    void update_posPlane();	// SBR
    /*!
     * \brief Called when the plane moved. Update the saved location and heading.
     */
    void update_posShip();	// SBR

signals:
    void confirmLanding();      //!< Signal to start landing process
    void confirmCancel();       //!< Signal to cancel landing proces and start loiter
};

#endif // ShipLANDING_H
