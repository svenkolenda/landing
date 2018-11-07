#ifndef TP_LANDING_H
#define TP_LANDING_H

/*-Includes-------------------------------------------------------------------*/

#include <stdint.h>

#include <QObject>
#include <QMessageBox>
#include <QTimer>

#include "QGCApplication.h"

/*-Local defines--------------------------------------------------------------*/

// TODO: struct vs QGeoCoordinate (?) SKO
struct __GPS
{
   int x;
   int y;
};

/*-TP_Landing-----------------------------------------------------------------*/

class TP_Landing : public QObject
{
   Q_OBJECT

public:
   // TODO: Singleton APF
   explicit TP_Landing(QObject *parent = nullptr);
   ~TP_Landing();

public slots:
   void prepareToLoiter();

private:
   QGCApplication* qgc = QGCApplication::_app;
   QTimer *timerLoiter = new QTimer(this);
   
   __GPS plane;		// SKO
   __GPS ship;		// SKO
   uint16_t distance;

   bool loiterShip_con = false;
   bool prepareToLand_con = false;

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

#endif // TP_LANDING_H
