//Simulation Schiff Position

#pragma once

#include <QtPositioning/qgeopositioninfosource.h>
#include "QGCToolbox.h"
#include <QTimer>

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

    // items for simulating QGC movement in jMAVSIM
    int32_t lat_int;
    int32_t lon_int;

    int _step_cnt;
    int _simulate_motion_index;
    bool _simulate_motion;
    float _rotation;

};

