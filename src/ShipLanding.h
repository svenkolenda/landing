#ifndef ShipLANDING_H
#define ShipLANDING_H

//-Includes------------------------------------------------------------------//

#include <stdint.h>
#include <math.h>
#include <deque>

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
#include "PX4FirmwarePlugin.h"

//-Local defines-------------------------------------------------------------//

Q_DECLARE_LOGGING_CATEGORY(ShipLandingLog)

#ifndef SHIP_LANDING_GPS
#define SHIP_LANDING_GPS
/**
 * @brief Struct to save horizontal and vertical distance relative to an object
 */
typedef struct __Distance
{
    double vertical_distance;
    double horizontal_distance;
} _Distance;

/**
 * @brief Struct to save the heading together with a timestamp
 *
 */
typedef struct __Heading
{
    double heading;
    QDateTime timestamp;
} _Heading;

/**
 * @brief Struct to save the coordinates and the direction of plane/ship
 *
 */
typedef struct __GPS
{
    QGeoCoordinate coord;
    double dir = 0; // TODO: Take out
    std::deque <_Heading> hdng_his; // heading history
} _GPS;
#endif

#ifndef HAVE_ENUM_SHIP_LANDING_STATE
#define HAVE_ENUM_SHIP_LANDING_STATE
typedef enum SHIP_LANDING_STATE
{
    IDLE,                           // 0
    MISSION,                        // 1
    RETURN,                         // 2
    LAND_REQ,                       // 3
    LAND_SEND,                      // 4
    LAND_APPROACH,                  // 5
    FALLBACK_RESTART_APPROACH,      // 6
    FALLBACK_RESTART_LOITER,        // 7
    FALLBACK_GO_AROUND,             // 8
    FALLBACK_PNR                    // 9
} SHIP_LANDING_STATE;
#endif

//-ShipLanding---------------------------------------------------------------//
/*!
 * \brief The ShipLanding class
 */
class ShipLanding : public QObject
{
    Q_OBJECT

private:    // attributes
    static ShipLanding* _instance;              //!< Singleton instance
    Vehicle* _vehicle = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    QTimer* timerObserve = new QTimer(this);    //!< Timer to observe landing
    double dir_miss;                            //!< Heading Mission landing
    __GPS ship;                                 //!< Information struct of ship
    SHIP_LANDING_STATE state = IDLE;            //!< State of the Shiplanding
    bool landReq = false;                       //!< Input of observeState
    bool landCancel = false;                    //!< Input of observeState

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
     * \brief Connect landing and cancel, connect PositionManger ship, configure timer.
     * \param parent QObject parent from the ShipLanding object
     */
    ShipLanding(QObject *parent = nullptr);
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
    QGeoCoordinate calcPosRelativeToShip(double, unsigned int, double);

    /*!
     * \brief calculate vertical and horizontal distance relative to given
     * coordinates and heading, with coordinates assumed to be (0,0) and heading
     * as y-axis.
     *                      P (Reference object)
     *                      |
     *                      | vertical distance
     *                      |
     * (other object)       |
     * A--------------------F
     *   horizontal distance
     * \param longitude of reference object
     * \param latitude of reference object
     * \param heading of reference object in degrees
     * \param longitude of other object
     * \param latitude of other object
     * \return Struct containing the distances.
     */
    _Distance calcDistanceRelativeTo(double, double, double, double, double);

    /*!
     * \brief Calculate heading rate.
     * \return Heading rate of the ship
     */
    double calcHeadingRate();
    /*!
     * \brief Calculate heading difference between ship and plane
     * \return Heading difference between ship and plane
     */
    double calcHeadingDiff();
    /*!
     * \brief Check if the ship is in a certain corridor so the plane can land.
     * \return Perpendicular deviation from ideal path.
     */
    double checkShipPosDif();
    bool checkPlanePos();

    /*!
     * \brief Check if plane is in a certain area behind the ship.
     * \return True if plane is in area where a landing approach is possible.
     */
    bool check_planeNearHomePoint();
    /*!
     * \brief Check if the rate of the ship heading is greater than MAX_HDNG_RATE.
     * \return true if okay
     */
    bool check_shipHeadingRate();
    /*!
     * \brief Check if the difference of the ship heading to the mission heading
     * is greater than MAX_HDNG_DIFF.
     * \return true if okay
     */
    bool check_shipHeadingDifference();
    /*!
     * \brief Check if the ship drove over the last wp of the landing mission.
     * \return true if okay
     */
    bool check_shipDroveOverLastWP();
    /*!
     * \brief Check if the distance ship to plane is greater than MAX_DISTANCE.
     * \return true if okay
     */
    bool check_maxDistShipToPlane();

public slots:
    /*!
     * \brief Reaction to init the landing. Called by UI.
     */
    Q_INVOKABLE void landingInit();
    /*!
     * \brief Reaction to start the landing. Called by UI.
     */
    Q_INVOKABLE void landingStart();
    /*!
     * \brief Reaction to cancel the Landing. Called by UI.
     */
    Q_INVOKABLE void landingCancel();

private slots:
    /*!
     * \brief Send the loiter point behind ship as home.
     */
    void send_homePoint();
    /*!
     * \brief GoTo the fallback point.
     */
    void send_fallbackGoAround();
    /*!
     * \brief Build and send the landing mission.
     */
    void send_landMission();
    /*!
     * \brief Build and send the landing geofence.
     */
    void send_geofence();

    /*!
     * \brief Observe ship heading, ship position relative to wp2 (in front of ship). Send mission if needed.
     */
    void observe_state();

    /*!
     * \brief Called when the plane moved. Update the saved location and heading.
     */
    void update_posShip(QGeoPositionInfo update);
};

#endif // ShipLANDING_H
