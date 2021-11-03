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
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
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
    QRect dimension( -5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this , dimension);
    this->resize(900,450);
    robot_polygon= viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon=new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);

    try{
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
    }
    catch(const Ice::Exception &e){std::cout<<e.what()<<std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	
}




//////////CODIGO DE LA MAQUINA DE ESTADOS//////////////////
void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    RoboCompLaser::TLaserData ldata ;
    int tam = 0;
    try {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
       // tam = (ldata.size())/4;
        //sort laser data from small to large distances using a lambda function.
       // std::sort( ldata.begin()+tam, ldata.end()-tam, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
        //previterangle = angle
    } catch (const Ice::Exception &e) {
        std::cout<<e.what()<<std::endl;
    }


    try{

        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha*180/M_PI);
        robot_polygon->setPos(bState.x, bState.z);

    } catch (const Ice::Exception &e) {
        std::cout<<e.what()<<std::endl;
    }
    target_to_robot = world_to_robot(bState);

    switch (estado) {

        //ESTADO EN ESPERA
        case State::IDLE:
            std::cout<<"estoy en estado IDLE"<<std::endl;
            try {
                differentialrobot_proxy->setSpeedBase(0,0);
            }catch (const Ice::Exception &e){
                std::cout<<e.what()<<std::endl;
            }
            if(target.active)
            {
                function.robot = QPointF(bState.x, bState.z);
                estado = State::RUN;

            }
                /////ALMACENAMOS LA POSICION ACTUAL DEL ROBOT AL ACTUALIZAR EL PUNTO////
            break;
            //EL IR PARA ADELANTE, EL CLASICO
        case State::RUN:
            std::cout<<"estoy en estado RUN"<<std::endl;
            estado = run(bState,  ldata);
            break;
            //REORIENTAR Y GIRAR EN LA PARED
        case State::OBSTACLE:
            std::cout<<"estoy en estado OBSTACLE"<<std::endl;
            estado = obstacle(bState, ldata);
            break;
            //UNA VEZ GIRADO, TIRAR ADELANTE HACIA EL PUNTO
        case State::SURROUND:
            std::cout<<"estoy en estado SURROUND"<<std::endl;
            estado = surround(bState, ldata);
            break;
    }


}

SpecificWorker::State SpecificWorker::run(const RoboCompGenericBase::TBaseState &bState,  const RoboCompLaser::TLaserData  &ldata){
    float distance = 0.0;
    float threshold = 300.0;
    int semiwidth = 16;
    Eigen::Vector2f robot_eigen(bState.x, bState.z);
    Eigen::Vector2f target_eigen(target.destiny.x(), target.destiny.y());

    ///////CONDITION TARGET EXIT///////
    if( distance = (target_eigen - robot_eigen).norm(); distance <= 100){
        target.active=false;
        return State::IDLE;
    }
    ///////CONDITION CRASH OBSTACLE///////
    if( distance_ahead(ldata, 300, 16))
    {
        try
        {
            differentialrobot_proxy->setSpeedBase(0,0);
        } catch (const Ice::Exception &e) {
            std::cout<<e.what()<<std::endl;
        }
        return State::OBSTACLE;
    }

    /////////////////////////////////////////////////

    //OJO A ESTO, AQUÍ LOS EJES A ATAN2 SE LOS PASA CAMBIADOS, PRIMERO EJE Y, LUEGO EJE X
    float beta = atan2(target_to_robot.x(), target_to_robot.y());
    float adv = MAX_ADV_SPEED * dist_to_target_object(distance)* dist_to_target_object(get_min_ldata_element(ldata,semiwidth).dist) *rotation_speed(beta);

    try
    {
        differentialrobot_proxy->setSpeedBase(adv,beta);
    } catch (const Ice::Exception &e)
    {
        std::cout<<e.what()<<std::endl;
    }
    return State::RUN;
}


SpecificWorker::State SpecificWorker::obstacle(const RoboCompGenericBase::TBaseState &bState,const RoboCompLaser::TLaserData &ldata ){
    float distance;
    float threshold=300.0;
    int semiwidth = 16;
    Eigen::Vector2f robot_eigen(bState.x, bState.z);
    Eigen::Vector2f target_eigen(target.destiny.x(), target.destiny.y());

    ///////CONDITION TARGET EXIT///////
    if( distance = (target_eigen - robot_eigen).norm(); distance <= 100){
        target.active;
        return State::IDLE;
    }


    ////CONDICION DE SALIDA A COMPROBACION DE ANGULO////
    if(distance_ahead(ldata,threshold,semiwidth))
    {
        try {
            differentialrobot_proxy->setSpeedBase(0,0);
        } catch (const Ice::Exception &e) {
        }
        return State::SURROUND;
    }
    /////FUNCION LMBDA PARA SUMA PARA MEDIA////
    float halftam= ldata.size()/2;

    /////GIRO///////
    try {
        differentialrobot_proxy->setSpeedBase(0,1);
        differentialrobot_proxy->setSpeedBase(0,1);
    } catch (const Ice::Exception &e) {

    }
    return State::OBSTACLE;
}

SpecificWorker::State SpecificWorker::surround(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData  &ldata) {
    float distance;
    Eigen::Vector2f robot_eigen(bState.x, bState.z);
    Eigen::Vector2f target_eigen(target.destiny.x(), target.destiny.y());
    if( distance = (target_eigen - robot_eigen).norm(); distance <= 100){
        target.active;
        return State::IDLE;
    }
    //METODO PARA COMPROBAR SI EL PUNTO EXISTE DENTRO DEL POLÍGONO DEL LASERs
    if (poly.containsPoint(target.destiny, Qt::OddEvenFill))
        return State::RUN;


    return State::SURROUND;
}
/////////////////////////////////////////////





////////////METODOS DE PROPOSITO GENERAL/////////////
/*
 * METODO PARA CALCULAR VELOCIDAD MÁXIMA SEGÚN DISTANCIA AL OBJETO
 *
 * V = Vmax * F(dist) * F(angle)
 *
 * F(Dist): Si dist > umbral == VMAX
 * si no: aplicamos: Vmax/umbral * Dist
 * F(Dist) = V/Vmax(METODO DIST TO TARGET)
 *
 *
 *
 *
 * F(angle):lny = e ^ x²/lambda
 *lambda = -(x²)/lnF(angle) (Rotation_Speed)
 * lambda = -(0,5²)/ln0.4
 */
float SpecificWorker::dist_to_target_object(float dist){
    float threshold = 500;
    float speed;
    if(dist >= threshold){
        speed = MAX_ADV_SPEED;
    }else{
        speed = dist * (MAX_ADV_SPEED/threshold);
    }
    return (speed / MAX_ADV_SPEED);
}
/*
 *
 *
 */
float SpecificWorker::rotation_speed(float beta) {
    float v= 0.0;
    float e = 2.71828;
    float lambda= 0.10857362;

    v = pow(e, -pow(beta,2)/lambda);

return v;
}

bool SpecificWorker::distance_ahead(const RoboCompLaser::TLaserData &ldata, float dist, int semiwidth)
{
    size_t s = ldata.size();
    auto min = std::min_element(ldata.begin() + (s/2 - semiwidth), ldata.end() - (s/2 - semiwidth), [](auto a, auto b){return a.dist < b.dist;});
    return (*min).dist <= dist;
}


RoboCompLaser::TData SpecificWorker::get_min_ldata_element(const RoboCompLaser::TLaserData &ldata, int semiwidth)
{
    size_t s = ldata.size();
    auto minimal_element = std::min_element(ldata.begin() + (s/2 - semiwidth), ldata.end() - (s/2 - semiwidth), [](auto a, auto b){return a.dist < b.dist;});
    return (*minimal_element);
}

QPointF SpecificWorker::world_to_robot(Target target, RoboCompGenericBase::TBaseState bState)
{
    float angle = bState.alpha;
    Eigen::Vector2f T(bState.x, bState.z), point_in_world(target.destiny.x(), target.destiny.y());
    Eigen::Matrix2f R;

    R << cos(angle), -sin(angle), sin(angle), cos(angle);

    Eigen::Vector2f point_in_robot = R.transpose() * (point_in_world - T);
    return QPointF(point_in_robot.x(), point_in_robot.y());


}
void SpecificWorker::new_target_slot(QPointF point) {
    target.destiny = point;
    function.target = target.destiny;
    target.active = true;
    qInfo() << target.destiny;
    estado= State::IDLE;

}
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
   static QGraphicsItem *laser_polygon = nullptr;
   //delete laser_polygon;
   // code to delete any existing laser graphic element
   if(laser_polygon!= nullptr){
       viewer->scene.removeItem(laser_polygon);
   }

   float cx, cy;
   poly.clear();
   poly << QPointF(0,0);
   for (auto &pointer : ldata){
        //transformar base
        //meter en poly

        cx=cos(pointer.angle)*pointer.dist;
        cy=sin(pointer.angle)*pointer.dist;
        poly << QPointF(cy,cx);

   }
   //poly.translate(last_point);
   // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's  // reference system
   QColor color("LightGreen");
   color.setAlpha(40);
   laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
   laser_polygon->setZValue(3);
}
bool SpecificWorker::isInFunction(const RoboCompGenericBase::TBaseState &bState)
{




    return false;
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

