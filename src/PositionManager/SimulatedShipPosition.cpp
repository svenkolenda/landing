#include "SimulatedShipPosition.h"

SimulatedShipPosition::SimulatedShipPosition()
    : QGeoPositionInfoSource(nullptr),
      lat_int(int(37.0*1e7)),          // Zuerich See 47.365464 // Augsburg HS 48.354772 // Malta/Lybien 34.416907
      lon_int(int(12.0*1e7)),           // Zuerich See 8.542199 // Augsburg HS 10.904693 // NMalte/Lybien 14.097748
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
    update_timer.setSingleShot(false);
    update_timer.start(minimumUpdateInterval());
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
    static double heading = 180.0, change = 0.0, last_change = 0.0;
    static int i = 0, max = 10;
    if (i++ == max)
    {
        last_change = change;
        change = static_cast<double>((qrand()%181)) * ((qrand()%2) == 1 ? 1 : -1) / 64;    // max change ~2.8°
        if (fabs(last_change) > 0.2 || fabs(change) < 0.1)
        {
            change = 0.0;
            max = 120;  // 120s = 2min
        }
        else
            max = (qrand()%3 + 1) * 5;  // 5s, 10s, 15s
        i = 0;
    }
    if (heading < 0)
        heading += 360;
    else if (heading >= 360)
        heading = fmod(heading, 360);
    heading += change;
    lat_int += LAT_LON_MOV * cos(heading * M_PI / 180);
    lon_int += LAT_LON_MOV * sin(heading * M_PI / 180);

    double longitude = static_cast<double>(lon_int) * 1e-7;
    double latitude = static_cast<double>(lat_int) * 1e-7;

    QGeoPositionInfo info(QGeoCoordinate(latitude, longitude), QDateTime::currentDateTime());

    info.setAttribute(QGeoPositionInfo::Attribute::Direction, heading);
    info.setAttribute(QGeoPositionInfo::Attribute::GroundSpeed, 4);

    lastPosition = info;
    emit positionUpdated(info);
}

QGeoPositionInfoSource::Error SimulatedShipPosition::error() const
{
    return QGeoPositionInfoSource::NoError;
}
