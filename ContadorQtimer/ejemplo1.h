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
        void increaseDisplay();

    private:
        QTimer *timer;
        int cont = 0;
};

#endif // ejemplo1_H
