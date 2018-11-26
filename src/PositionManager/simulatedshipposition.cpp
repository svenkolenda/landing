#include "simulatedshipposition.h"

#include <QDate>
#include <QtCore>
#include <QDateTime>

SimulatedShipPosition::SimulatedShipPosition()
    : QGeoPositionInfoSource(NULL),
      lat_int(47.3978691*1e7),          //gerade Zürich - Augsburg 48.354096
      lon_int(8.54495665*1e7),           //gerade Zürich - Augsburg 10.842208
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

void SimulatedShipPosition::updatePosition()
{
    //270 für 15 km/h einstellen, bzw mit Kurve variieren
    int32_t lat_mov = 300; //Längengrad - größer Richtung Norden
    int32_t lon_mov = 150; //Breitengrad - größer Richtung Osten

    lat_int -= lat_mov; // + oben / - unten
    lon_int += lon_mov; // + rechts / - links

    double longitude = ((double) (lon_int))*1e-7;
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
