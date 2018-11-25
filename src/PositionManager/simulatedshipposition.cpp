#include "simulatedshipposition.h"

#include <QDate>
#include <QtCore>
#include <QDateTime>

SimulatedShipPosition::SimulatedShipPosition()
    : QGeoPositionInfoSource(NULL),
      lat_int(47.397042*1e7),          //gerade Zürich - Augsburg 48.354096
      lon_int(8.544124*1e7),           //gerade Zürich - Augsburg 10.842208
      _step_cnt(0),
      _simulate_motion_index(0),
      _simulate_motion(true),
      _rotation(0.0F)
{
    QDateTime currentDateTime = QDateTime::currentDateTime();

    qsrand(currentDateTime.toTime_t());

    connect(&update_timer, &QTimer::timeout, this, &SimulatedShipPosition::updatePosition);
}

QGeoPositionInfo SimulatedShipPosition::lastKnownPosition(bool /*fromSatellitePositioningMethodsOnly*/) const
{
    return lastPosition;
}

SimulatedShipPosition::PositioningMethods SimulatedShipPosition::supportedPositioningMethods() const
{
    return AllPositioningMethods;
}

int SimulatedShipPosition::minimumUpdateInterval() const
{
    return 1000;
}

void SimulatedShipPosition::startUpdates()
{
    int interval = updateInterval();
    if (interval < minimumUpdateInterval())
        interval = minimumUpdateInterval();

    update_timer.setSingleShot(false);
    update_timer.start(interval);
}

void SimulatedShipPosition::stopUpdates()
{
    update_timer.stop();
}

void SimulatedShipPosition::requestUpdate(int /*timeout*/)
{
    emit updateTimeout();
}

int SimulatedShipPosition::getRandomNumber(int size)
{
    if(size == 0) {
        return 0;
    }

    int num = (qrand()%2 > 1) ? -1 : 1;

    return num*qrand()%size;
}

void SimulatedShipPosition::updatePosition()
{
    int32_t lat_mov = 1;
    int32_t lon_mov = 0;

    //_rotation += (float) .1;
    //lat_mov = _simulated_motion[_simulate_motion_index].lat;
    //lon_mov = _simulated_motion[_simulate_motion_index].lon*sin(_rotation);

    lon_int += lat_mov;
    lat_int += lon_mov;

    //double longitude = ((double) (lon_int + getRandomNumber(250)))*1e-7;
    double longitude = ((double)  (lon_int))*1e-7;

    //double latitude = ((double)  (lat_int + 0.0001))*1e-7;
    double latitude = ((double) (lat_int))*1e-7;

    QDateTime timestamp = QDateTime::currentDateTime();

    QGeoCoordinate position(latitude, longitude); //Koordinaten setzen
    QGeoPositionInfo info(position, timestamp); //weitergeben davon

    if(lat_mov || lon_mov) {
        info.setAttribute(QGeoPositionInfo::Attribute::Direction, 3.14/2);
        info.setAttribute(QGeoPositionInfo::Attribute::GroundSpeed, 5);
    }

    lastPosition = info;

    emit positionUpdated(info);
}

QGeoPositionInfoSource::Error SimulatedShipPosition::error() const
{
    return QGeoPositionInfoSource::NoError;
}
