/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
  const float threshold = 290;// millimeters
    //float rot = 0.2618; // rads per second
    //float neg_rot_45 = -0.785398; 349066
    //float rot_45 = 0.785398; // 45 degrees angle
    int tam;
    int turn = 0;
    float angle = 0;
    //float previterangle = 0;
 
    try
    {
    	// read laser data 
        
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 
        tam = (ldata.size())/3;
	//sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin()+tam, ldata.end()-tam, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });  
        //previterangle = angle;
        angle = ldata[tam].angle;
        
        turn = 1;
        if(angle > 0){
            turn = -1;
        }
        
        
        
        
        if( ldata[tam].dist < threshold)
        {
            std::cout << ldata.front().dist << std::endl;
            differentialrobot_proxy->setSpeedBase(0, 2 * turn);
            usleep((rand()%(785398-261800  + 1)+261800)/2);  // random wait between 1.5s and 0.1sec            
        }    
        else
        {
            
            if(ldata[tam].dist > 900){
                differentialrobot_proxy->setSpeedBase(999, 0); 
            }else if(ldata[tam].dist > 700){
                differentialrobot_proxy->setSpeedBase(699, 0); 
            }else if(ldata[tam].dist > 600){
                differentialrobot_proxy->setSpeedBase(599, 0); 
            }else if(ldata[tam].dist > 500){
                differentialrobot_proxy->setSpeedBase(499, 0);
            }else if(ldata[tam].dist > 300){
                differentialrobot_proxy->setSpeedBase(200, 0); 
            }
            
        }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

