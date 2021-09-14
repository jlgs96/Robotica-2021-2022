#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
	
	mytimer.connect(std::bind(&ejemplo1::cuenta, this));
    mytimer.start(1000);
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
		mytimer.start(1000);
	qDebug() << "click on button";
}

void ejemplo1::cuenta()
{
    lcdNumber->display(++cont);
	trick++;
}

