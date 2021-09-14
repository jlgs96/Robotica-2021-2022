#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	timer = new QTimer(this);
    setupUi(this);
	show();
 
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    connect(timer, SIGNAL(timeout()), this, SLOT(increaseDisplay()));
    timer->start(1000);
}

void ejemplo1::doButton()
{

    if(timer->isActive())
        timer->stop();
    else
        timer->start(1000);

	qDebug() << "click on button";
}
void ejemplo1::increaseDisplay() {
    lcdNumber->display(++cont);
}



