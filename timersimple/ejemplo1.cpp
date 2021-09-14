#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();

    horizontalSlider->setMaximum(5000);
    horizontalSlider->setMinimum(0);
    horizontalSlider->setValue(500);
    value_h = horizontalSlider->value();
    lcdNumberPeriod->display(horizontalSlider->value());


	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(doSlide()));


	mytimer.connect(std::bind(&ejemplo1::cuenta, this));
    TimereTime.connect(std::bind(&ejemplo1::elapsedTime, this));
    TimereTime.start(1000);
    mytimer.start(500);
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	static bool stopped = false;
	stopped = !stopped;
	if(stopped)
		mytimer.stop();
	else
		mytimer.start(horizontalSlider->value());
	qDebug() << "click on button";
}
void ejemplo1::doSlide()
{
    if(value_h != horizontalSlider->value()){
        lcdNumberPeriod->display(horizontalSlider->value());
        value_h = horizontalSlider->value();
        mytimer.setPeriod(value_h);
    }
}
void  ejemplo1::elapsedTime()
{
    lcdNumbereTime->display(lcdNumbereTime->intValue()+1);
}
void ejemplo1::cuenta()
{
    lcdNumber->display(++cont);
	trick++;
}

