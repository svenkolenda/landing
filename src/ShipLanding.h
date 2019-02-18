#ifndef SHIPLANDING_H
#define SHIPLANDING_H

//-Includes---------------------------------------------------------------------------------------//

//C++
#include <stdint.h>
#include <math.h>
#include <deque>

//Qt
#include <QObject>
#include <QTimer>
#include <QGeoCoordinate>

//QGC
#include "QGCApplication.h"
#include "QGCLoggingCategory.h"
#include "PositionManager.h"
#include "Vehicle.h"
#include "MissionManager.h"
#include "QmlObjectListModel.h"
#include "QGCFencePolygon.h"
#include "PX4FirmwarePlugin.h"

//-Local defines----------------------------------------------------------------------------------//

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

#endif //SHIP_LANDING_GPS

#ifndef HAVE_ENUM_SHIP_LANDING_STATE
#define HAVE_ENUM_SHIP_LANDING_STATE

typedef enum SHIP_LANDING_STATE
{
    IDLE = 0,                       // 0
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

#endif //HAVE_ENUM_SHIP_LANDING_STATE

//-ShipLanding------------------------------------------------------------------------------------//

/*!
 * \brief The ShipLanding class
 */
class ShipLanding : public QObject
{
    Q_OBJECT

private:    // attributes
    static ShipLanding* _instance;                  //!< Singleton instance
    Vehicle* _vehicle        = qgcApp()->toolbox()->multiVehicleManager()->activeVehicle();
    QTimer* timerObserve     = new QTimer(this);    //!< Timer to observe landing
    double dir_miss;                                //!< Heading Mission landing
    __GPS ship;                                     //!< Information struct of ship
    __GPS water;                                    //!< Information struct of water point
    SHIP_LANDING_STATE state = IDLE;                //!< State of the Shiplanding
    bool landReq             = false;               //!< Input of observeState
    bool landWater           = false;               //!< Mealy-Input of observeState
    bool landCancel          = false;               //!< Input of observeState
    bool landReset           = false;               //!< Input of observeState
    QList<QGeoCoordinate> wp;                       //!< Waypoint list

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
     * \param altitude planned absolute alltitude
     * \param dDir wanted direction change
     * \return pos
     */
    QGeoCoordinate calcPosRelativeToCoord(double distance, unsigned int altitude, QGeoCoordinate coord, double dDir);

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
     * \param x_p longitude of reference object
     * \param y_p latitude of reference object
     * \param hdg heading of reference object in degrees
     * \param x_a longitude of other object
     * \param y_a latitude of other object
     * \return dist struct containing the distances.
     */
    _Distance calcDistanceRelativeTo(double x_p, double y_p, double hdg, double x_a, double y_a);

    /*!
     * \brief Calculate heading rate.
     * \return heading_rate Heading rate of the ship
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
    double calcShipPosDif();

    /*!
     * \brief Check if plane is in a certain area behind the ship.
     * \return True if plane is in area where a landing approach is possible.
     */
    bool checkPlaneNearHomePoint();

    /*!
     * \brief Check if ship is in the landing corridor of the mission.
     * \return true if in landing corridor
     */
    bool checkShipInLandingCorridor();

    /*!
     * \brief Check if water point is in the landing corridor of the mission.
     * \return true if in landing corridor
     */
    bool checkWaterInLandingCorridor();

    /*!
     * \brief Check if the rate of the ship heading is greater than MAX_HDNG_RATE.
     * \return True if okay
     */
    bool checkShipHeadingRate();

    /*!
     * \brief Check if the difference of the ship heading to the mission heading
     * is greater than MAX_HDNG_DIFF.
     * \return True if okay
     */
    bool checkShipHeadingDifference();

    /*!
     * \brief Check if the ship drove over the last wp of the landing mission.
     * \return True if okay
     */
    bool checkShipDroveOverLastWP();

    /*!
     * \brief Check if the distance ship to plane is greater than MAX_DISTANCE.
     * \return True if okay
     */
    bool checkMaxDistShipToPlane();

    /*!
     * \brief Check if the distance water point to plane is greater than MAX_DISTANCE.
     * \return True if okay
     */
    bool checkMaxDistWaterToPlane();

    /*!
     * \brief Check if the distance ship to plane is greater than MAX_DISTANCE.
     * \return True if okay
     */
    bool checkDistShipToHome();

public slots:

    /*!
     * \brief Reaction to init the landing. Called by UI.
     */
    Q_INVOKABLE void landingInit();

    /*!
     * \brief Reaction to start the landing on ship. Called by UI.
     */
    Q_INVOKABLE void landingStartRtS();

    /*!
     * \brief Reaction to start the landing on water. Called by UI.
     */
    Q_INVOKABLE void landingStartRtW();

    /*!
     * \brief Reaction to cancel the landing. Called by UI.
     */
    Q_INVOKABLE void landingCancel();

    /*!
     * \brief Reaction to reset the statemachine. Called by UI.
     */
    Q_INVOKABLE void landingReset();

private slots:

    /*!
     * \brief Send the loiter point behind ship as home.
     */
    void sendHomePoint();

    /*!
     * \brief Send the loiter point behind ship as home.
     */
    void sendHomeWaterPoint();

    /*!
     * \brief GoTo the home point.
     */
    void sendHomeGoto();

    /*!
     * \brief GoTo the fallback point for the ship.
     */
    void sendFallbackGoAround();

    /*!
     * \brief GoTo the fallback point for the water point.
     */
    void sendFallbackGoAroundWater();

    /*!
     * \brief Build and send the landing mission for the ship.
     */
    void sendLandMission(int idx);

    /*!
     * \brief Build and send the landing mission for the water point.
     */
    void sendLandMissionWater(int idx);

    /*!
     * \brief Build and send the landing geofence for the ship.
     */
    void sendGeofence();

    /*!
     * \brief Build and send the landing geofence for the water point.
     */
    void sendGeofenceWater();

    /*!
     * \brief Observe ship heading, ship position relative to wp2 (in front of ship).
     * Send mission if needed.
     */
    void observeState();

    /*!
     * \brief Called when the plane moved. Update the saved location and heading.
     */
    void updatePosShip(QGeoPositionInfo update);
};

#endif // SHIPLANDING_H

//-EOF--------------------------------------------------------------------------------------------//
