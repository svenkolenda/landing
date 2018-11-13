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

struct __GPS
{
    QGeoCoordinate coord;
    double dir;
};

/*-ShipLanding-----------------------------------------------------------------*/

class ShipLanding : public QObject
{
    Q_OBJECT

public:
    static void release();
    static QObject* qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine);

public slots:
    Q_INVOKABLE void initDialog();
    Q_INVOKABLE void cancelDialog();

private:
    //attributes
    static ShipLanding* _instance;

    QTimer* timerLoiter = new QTimer(this);

    Vehicle*  _vehicle;

    __GPS plane;
    __GPS ship;
    uint16_t distance;

    bool loiterShip_con = false;
    bool prepareToLand_con = false;

    // functions
    explicit ShipLanding(QObject *parent = nullptr);
    ~ShipLanding();

    QGeoCoordinate calcLoiterPos();

    void start_timerLoiter();
    void stop_timerLoiter();

private slots:
    void prepareToLoiter();
    void loiterShip();
    void land();

    void update_posPlane();	// SBR
    void update_posShip();	// SBR

signals:
    void confirmLanding();
    void confirmCancel();

};

#endif // ShipLANDING_H
