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

/*-Local defines--------------------------------------------------------------*/

// TODO: struct  SKO
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

public slots:
    void prepareToLoiter();

private:
    //attributes
    static ShipLanding* _instance;

    QGCApplication* qgc = QGCApplication::_app;
    QTimer* timerLoiter = new QTimer(this);

    Vehicle*  _vehicle;

    __GPS plane;		// SKO
    __GPS ship;		// SKO
    uint16_t distance;

    bool loiterShip_con = false;
    bool prepareToLand_con = false;

   // functions
    explicit ShipLanding(QObject *parent = nullptr);
    ~ShipLanding();

    void calculateDistance(__GPS, __GPS);	// APF

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
