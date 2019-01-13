//Simulation Schiff Position

#ifndef SIMULATED_SHIP_POSITION_H
#define SIMULATED_SHIP_POSITION_H

#include <QtPositioning/qgeopositioninfosource.h>
#include <QTimer>
#include <QDate>
#include <QtCore>
#include <QDateTime>
#include "QGCToolbox.h"

// SeaWatch3: avg: 7knots, max 8.1knots
const int LAT_LON_MOV = 300; //!< max movement

class SimulatedShipPosition : public QGeoPositionInfoSource
{
    Q_OBJECT

public:
    SimulatedShipPosition();

    QGeoPositionInfo lastKnownPosition(bool fromSatellitePositioningMethodsOnly = false) const;

    PositioningMethods supportedPositioningMethods() const;
    int minimumUpdateInterval() const;
    Error error() const;

public slots:
    virtual void startUpdates();
    virtual void stopUpdates();

    virtual void requestUpdate(int timeout = 5000);

private slots:
    void updatePosition();

private:
    QTimer update_timer;
    QGeoPositionInfo lastPosition;

    int lat_int;
    int lon_int;

    int _step_cnt;
    int _simulate_motion_index;
    bool _simulate_motion;
    float _rotation;

};

#endif
