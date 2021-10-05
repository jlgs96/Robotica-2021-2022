/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *button;
    QLCDNumber *lcdNumber;
    QLCDNumber *lcdNumbereTime;
    QLCDNumber *lcdNumberPeriod;
    QSlider *horizontalSlider;
    QLineEdit *lineEdit;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName(QString::fromUtf8("Counter"));
        Counter->resize(700, 400);
        button = new QPushButton(Counter);
        button->setObjectName(QString::fromUtf8("button"));
        button->setGeometry(QRect(130, 140, 131, 41));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setGeometry(QRect(50, 40, 301, 91));
        lcdNumbereTime = new QLCDNumber(Counter);
        lcdNumbereTime->setObjectName(QString::fromUtf8("lcdNumbereTime"));
        lcdNumbereTime->setGeometry(QRect(372, 40, 301, 91));
        lcdNumberPeriod = new QLCDNumber(Counter);
        lcdNumberPeriod->setObjectName(QString::fromUtf8("lcdNumberPeriod"));
        lcdNumberPeriod->setGeometry(QRect(50, 220, 301, 91));
        horizontalSlider = new QSlider(Counter);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(100, 320, 181, 51));
        horizontalSlider->setOrientation(Qt::Horizontal);
        lineEdit = new QLineEdit(Counter);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(440, 140, 151, 27));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QApplication::translate("Counter", "Counter", nullptr));
        button->setText(QApplication::translate("Counter", "START / STOP", nullptr));
        lineEdit->setText(QApplication::translate("Counter", "Elapsed Time", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
