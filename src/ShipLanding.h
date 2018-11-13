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
    static ShipLanding* getInstance();
    static void release();
    QGeoCoordinate calcLoiterPos(); //TODO: public for testing

public slots:
    Q_INVOKABLE void prepareToLoiter();

private:
    //attributes
    static ShipLanding* _instance;

    //QGCApplication* qgc = QGCApplication::_app;     // qgcApp() delivers Singleton, delete qgc (?)
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

    //void calcDistance(__GPS, __GPS);	// QGeoCoord, distanceTo(coord)
    //QGeoCoordinate calcLoiterPos();   // TODO: public for testing

    void start_timerLoiter();
    void stop_timerLoiter();

    bool initDialog();		// JGE
    bool cancelDialog();	// JGE

private slots:
    void loiterShip();
    void land();
    void update_posPlane();	// SBR
    void update_posShip();	// SBR

signals:
    void initDialog_yes();
    void initDialog_no();
    void cancelDialog_yes();
    void cancelDialog_no();

};

#endif // ShipLANDING_H
