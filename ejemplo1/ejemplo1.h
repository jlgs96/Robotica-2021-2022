#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>
class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doButton();
        void doSlide();
        void elapsedTime();
        void increaseDisplay();

    private:
        QTimer *timer ;
        QTimer *timerETime;
    int cont = 0;
        int contetime =0;
        int value_h = 0;


};

#endif // ejemplo1_H
