#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	timer = new QTimer(this);
    timerETime = new QTimer(this);
    setupUi(this);
	show();

    horizontalSlider->setMaximum(5000);
    horizontalSlider->setMinimum(0);
    horizontalSlider->setValue(500);
    value_h = horizontalSlider->value();
    lcdNumberPeriod->display(horizontalSlider->value());

	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    connect(timer, SIGNAL(timeout()), this, SLOT(increaseDisplay()));
    connect(timerETime, SIGNAL(timeout()),this, SLOT(elapsedTime()));
    connect(horizontalSlider, SIGNAL(sliderReleased()),this, SLOT(doSlide()));
    timerETime->start(1000);
    timer->start(500);
}

void ejemplo1::doButton()
{

    if(timer->isActive())
        timer->stop();
    else
        timer->start(horizontalSlider->value());

	qDebug() << "click on button";
}
void ejemplo1::increaseDisplay() {
    lcdNumber->display(++cont);
}
void ejemplo1::doSlide() {
    if(value_h != horizontalSlider->value()){
        lcdNumberPeriod->display(horizontalSlider->value());
        value_h = horizontalSlider->value();
        timer->setInterval(value_h);
    }
}
void ejemplo1::elapsedTime() {

    lcdNumbereTime->display(++contetime);
}



