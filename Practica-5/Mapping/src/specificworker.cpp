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
    //////INICIALIZACIÓN DEL MAPP//////
    QRect dimension( -5200, -2700, 10400, 5400);
    viewer = new AbstractGraphicViewer(this , dimension);
    Mapp.initialize(dimension,TILE,&viewer->scene);
    this->resize(900,450);
    robot_polygon= viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon=new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);


    try{
        RoboCompGenericBase::TBaseState r_state;
        differentialrobot_proxy->getBaseState(r_state);
    }
    catch(const Ice::Exception &e){std::cout<<e.what()<<std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	
}




//////////CODIGO DE LA MAQUINA DE ESTADOS//////////////////
void SpecificWorker::compute() {

    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    RoboCompLaser::TLaserData ldata;
    int tam = 0;
    try {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        // tam = (ldata.size())/4;
        //sort laser data from small to large distances using a lambda function.
        // std::sort( ldata.begin()+tam, ldata.end()-tam, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
        //previterangle = angle
    } catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }


    try {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);

    } catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }
    static float alpha_Robot = 0.0;
    switch (mappState) {

        //ESTADO EN ESPERA
        case State::IDLE:
            std::cout<<"estoy en estado IDLE"<<std::endl;
            try
            {
                differentialrobot_proxy->setSpeedBase(0,0);
            }catch (const Ice::Exception &e){
                std::cout<<e.what()<<std::endl;
            }


        case State::EXPLORE:
            std::cout<<"estoy en estado EXPLORE"<<std::endl;
            try
            {
                mappState=exploringRoom(ldata,r_state);
            }catch (const Ice::Exception &e){
                std::cout<<e.what()<<std::endl;
            }
                /////ALMACENAMOS LA POSICION ACTUAL DEL ROBOT AL ACTUALIZAR EL PUNTO////
            break;
            //EL IR PARA ADELANTE, EL CLASICO
        case State::DOOR:
            std::cout<<"estoy en estado DOOR"<<std::endl;
            try
            {
                mappState=lookDoor(ldata,r_state);
            }catch (const Ice::Exception &e)
            {
                std::cout<<e.what()<<std::endl;
            }
            break;
            //REORIENTAR Y GIRAR EN LA PARED
        case State::CHANGEROOM:
            std::cout<<"estoy en estado CHANGEROOM"<<std::endl;
            //estado = obstacle(r_state, ldata);
            try
            {
                mappState = changeRoom(ldata,r_state);
            }catch (const Ice::Exception &e) {
                std::cout << e.what() << std::endl;
            }
            break;

    }



}
////UPDATE MAP////
void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float cx, cy;
    float end;
    float slice ;
    ////SLICE////
    float part;
    for (auto &ld: ldata)
    {
    ////CALCULO DE COORDENADAS DEL LASER////
        cx = ld.dist * cos(ld.angle);
        cy = ld.dist *sin(ld.angle);

        Eigen::Vector2f  laser_tip(cy,cx);
        part =ld.dist/TILE/2;
        end = 1.0 - (1/part);
        slice = 1/part;
        for(const auto &&point : iter::range<float>(0.0, end, slice))
        {
            auto q = robot_to_world(r_state, laser_tip * point);
            Mapp.add_miss(q);
        }
        auto rr = robot_to_world(r_state,laser_tip);
        if(ld.dist < MAX_LASER_DIST)
        {
            Mapp.add_hit(rr);
        }
        else
            Mapp.add_miss(rr);
    }
}


SpecificWorker::State SpecificWorker::exploringRoom(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    calculate_door_points(ldata, r_state);
   update_map(ldata, r_state);
   float percenteage_changed=0.0;
   percenteage_changed = Mapp.percentage_changed();

   std::cout << percenteage_changed*1000 << std::endl;
   percenteage_changed = percenteage_changed * 1000;

    ///EXIT CONDITION///
    if(percenteage_changed<1.2 )
    {
        try
        {
            differentialrobot_proxy->setSpeedBase(0,0);
        }catch (const Ice::Exception &e)
        {
            std::cout<<e.what()<<std::endl;
        }
        return State::DOOR;
    }
    ///MAPPEO///
   try
   {
        differentialrobot_proxy->setSpeedBase(0,0.5);
   }catch (const Ice::Exception &e)
   {
       std::cout<<e.what()<<std::endl;
   }

   return State::EXPLORE;
}


SpecificWorker::State SpecificWorker::lookDoor(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    int indexDoor = Doors.size()-1;
    int mov = 1500;
    ////COMPROBAR SI ULTIMO ELEMENTO PROCESADO---CONDICION DE SALIDA////
    if((int)Doors[indexDoor].midPointDoors.size()>0)
    {
        if(indexDoor = 0; (int)Doors[indexDoor].midPointDoors.size()>0)
        {
            return State::IDLE;
        }
    }

    Eigen::Vector2f middle = (Doors[indexDoor].p1 + Doors[indexDoor].p2)/2;

    ///PUERTA A PROCESAR////
    if(abs(Doors[indexDoor].p1[0] - Doors[indexDoor].p2[0]) < 100)
    {
        Doors[indexDoor].midPointDoors.push_back(Eigen::Vector2f(middle[0]+mov, middle[1]));
        Doors[indexDoor].midPointDoors.push_back(Eigen::Vector2f(middle[0]-mov, middle[1]));
    }

    if(abs(Doors[indexDoor].p1[1] - Doors[indexDoor].p2[1]) < 100)
    {
        Doors[indexDoor].midPointDoors.push_back(Eigen::Vector2f(middle[0], middle[1]+ mov));
        Doors[indexDoor].midPointDoors.push_back(Eigen::Vector2f(middle[0] , middle[1]- mov));

    }

    midPointsDoorGlobal = Doors[indexDoor].midPointDoors;

    Eigen::Vector2f  robot(r_state.x, r_state.y);
    std::sort(midPointsDoorGlobal.begin(), midPointsDoorGlobal.end(), [robot](Eigen::Vector2f a, Eigen::Vector2f b)
    {return(a-robot).norm() < (b-robot).norm();});
    return State::CHANGEROOM;
}

SpecificWorker::State SpecificWorker::changeRoom(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float distance = 0.0;

    Eigen::Vector2f robot_eigen(r_state.x, r_state.y);
    static int flag = 0;
    Eigen::Vector2f pointAux = world_to_robot(r_state, midPointsDoorGlobal[flag]);
    ///////CONDITION switch EXIT///////
    if(distance = pointAux.norm(); distance <= 150)
    {
        try
        {
            differentialrobot_proxy->setSpeedBase(0,0);
        }catch (const Ice::Exception &e)
        {
            std::cout<<e.what()<<std::endl;
        }
        flag++;
        return State::CHANGEROOM;
    }
    ////EXIT CONDITIONS////
    if(flag == 2)
    {
        flag = 0;
        return State::EXPLORE;
    }


    /////////////////////////////////////////////////



    //OJO A ESTO, AQUÍ LOS EJES A ATAN2 SE LOS PASA CAMBIADOS, PRIMERO EJE Y, LUEGO EJE X

    /*QPolygonF laser_poly;
    for(auto &&l : ldata)
        laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    auto [_, __, adv, rot, ___] = dw.compute(pointAux, laser_poly,
                                             Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                             Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                             nullptr /*&viewer_robot->scene*///);
    /*const float rgain = 0.8;
    float rotation = rgain*rot;

    float movement = MAX_ADV_SPEED * dist_to_target_object(distance) * rotation_speed(rotation);



    try
    {
        differentialrobot_proxy->setSpeedBase(movement,rotation);
    } catch (const Ice::Exception &e)
    {
        std::cout<<e.what()<<std::endl;
    }
    */
    moveRobot(ldata,r_state, distance, pointAux);

    return State::CHANGEROOM;
}


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
    float threshold = 1000;
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
    float lambda_no_drive = 0.36067376;
    v = pow(e, -pow(beta,2)/lambda_no_drive);

return v;
}

bool SpecificWorker::distance_ahead(const RoboCompLaser::TLaserData &ldata, float dist, int semiwidth)
{
    size_t s = ldata.size();
    auto min = std::min_element(ldata.begin() + (s/2 - semiwidth), ldata.end() - (s/2 - semiwidth), [](auto a, auto b){return a.dist < b.dist;});
    return (*min).dist <= dist;
}


void SpecificWorker::moveRobot(const RoboCompLaser::TLaserData &ldata,
                               const RoboCompFullPoseEstimation::FullPoseEuler &r_state, float d, Eigen::Vector2f v)
                               {
                                   QPolygonF laser_poly;
                                   for(auto &&l : ldata)
                                       laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
                                   auto [_, __, adv, rot, ___] = dw.compute(v, laser_poly,
                                                                            Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                                            Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                                            nullptr /*&viewer_robot->scene*/);
                                   const float rgain = 0.8;
                                   float rotation = rgain*rot;

                                   float movement = MAX_ADV_SPEED * dist_to_target_object(d) * rotation_speed(rotation);

                                   try
                                   {
                                       differentialrobot_proxy->setSpeedBase(movement,rotation);
                                   } catch (const Ice::Exception &e)
                                   {
                                       std::cout<<e.what()<<std::endl;
                                   }
                               }
bool SpecificWorker::distance_side(const RoboCompLaser::TLaserData &ldata, float dist, int iterBegin, int iterEnd)
{

    auto min = std::min_element(ldata.begin() + (iterBegin), ldata.end() - (iterEnd), [](auto a, auto b){return a.dist < b.dist;});
    return (*min).dist <= dist;
}
RoboCompLaser::TData SpecificWorker::get_min_ldata_element(const RoboCompLaser::TLaserData &ldata, int semiwidth)
{
    size_t s = ldata.size();
    auto minimal_element = std::min_element(ldata.begin() + (s/2 - semiwidth), ldata.end() - (s/2 - semiwidth), [](auto a, auto b){return a.dist < b.dist;});
    return (*minimal_element);
}
RoboCompLaser::TData SpecificWorker::get_max_ldata_element(const RoboCompLaser::TLaserData &ldata, int semiwidth)
{
    size_t s = ldata.size();
    auto maximun_element = std::max_element(ldata.begin() + (s/2 - semiwidth), ldata.end() - (s/2 - semiwidth), [](auto a, auto b){return a.dist > b.dist;});
    return (*maximun_element);
}


void SpecificWorker::calculate_door_points(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float RangeThreshold = 800;
    float isDoor = 700;
    float derivate = 0.0;
    float x, y;
    std::vector<int> indices (0);
    std::vector<Eigen::Vector2f> pointsDoor;
    Eigen::Vector2f points, mapDoor;


    for(auto &&[k, point]:iter::sliding_window(ldata, 2)|iter::enumerate)
    {
        derivate = point[1].dist - point[0].dist;
        RoboCompLaser::TData data;

        if(abs(derivate)>RangeThreshold)
        {
            if(derivate > RangeThreshold)
                data = ldata[k];
            if(derivate< -RangeThreshold)
                data = ldata[k+1];

            x = data.dist * sin(data.angle);
            y = data.dist * cos(data.angle);


            Eigen::Vector2f  tip(x,y);
            mapDoor = robot_to_world(r_state, tip);
            pointsDoor.push_back(mapDoor);
        }
    }
    for(auto &&c: iter::combinations_with_replacement(pointsDoor, 2))
    {
        if((c[0] - c[1]).norm()>500 and (c[0]-c[1]).norm()<1100)
        {
            Door daux(c[0],c[1]);
            if(auto res = std::find_if(Doors.begin(), Doors.end(), [daux](auto a){return daux == a;}); res == Doors.end())
                Doors.push_back(daux);
        }
    }
    paintDoor(pointsDoor);
}

void SpecificWorker::paintDoor(const std::vector<Eigen::Vector2f> &peaks)
{
    static std::vector<QGraphicsItem*> door_points;
    for(auto dp : door_points) viewer->scene.removeItem(dp);
    door_points.clear();
    /*for(const auto &p: peaks)
    {
        door_points.push_back(viewer->scene.addRect(QRectF(p.x()-100, p.y()-100, 200, 200),
                                                    QPen(QColor("Magenta")), QBrush(QColor("Magenta"))));
        door_points.back()->setZValue(200);
    }
    */
    for(const auto &d: Doors)
    {
        QPointF p1(d.p1.x(),d.p1.y()), p2(d.p2.x(), d.p2.y());
        door_points.push_back(viewer->scene.addLine(QLineF(p1,p2),
                                                    QPen(QColor("Blue"),50)));
        door_points.back()->setZValue(200);
    }

}

Eigen::Vector2f SpecificWorker::world_to_robot( RoboCompFullPoseEstimation::FullPoseEuler r_state, const Eigen::Vector2f &point_in_world)
{
    float angle = r_state.rz;
    Eigen::Vector2f T(r_state.x, r_state.y);
    Eigen::Matrix2f R;

    R << cos(angle), -sin(angle), sin(angle), cos(angle);

    Eigen::Vector2f point_in_robot = R.transpose() * (point_in_world - T);
    return point_in_robot;
}

Eigen::Vector2f  SpecificWorker::robot_to_world(const RoboCompFullPoseEstimation::FullPoseEuler &r_state, Eigen::Vector2f cartesianP)
{

    float angle = r_state.rz;
    Eigen::Matrix2f R;
    R << cos(angle), -sin(angle), sin(angle), cos(angle);

    Eigen::Vector2f robot(r_state.x,r_state.y);
    return R * cartesianP+robot;

}
void SpecificWorker::new_target_slot(QPointF point) {
    target.destiny = point;
    linear_function.target = target.destiny;
    target.active = true;
    qInfo() << target.destiny;
    mappState= State::EXPLORE;

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
bool SpecificWorker::isInFunction(const  RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{

    float A = linear_function.robot.y() - linear_function.target.y();
    float B = linear_function.target.x() - linear_function.robot.x();
    float C = (linear_function.robot.x() - linear_function.target.x())*linear_function.robot.y() +(linear_function.target.y() - linear_function.robot.y())*linear_function.robot.x();
    float dist = (abs(A*r_state.x + B*r_state.y + C))/ sqrt(pow(A,2) + pow(B,2));

    //SI NO ESTÁ EN LA FUNCIÓN (con cierto margen)
    if(abs(dist)>10)
    {
        return false;
    }

    //COMPROBAR SI LA POSICIÓN ACTUAL ES MEJOR QUE LA ÚLTIMA POSICIÓN
    Eigen::Vector2f robot_eigen(r_state.x, r_state.y);
    Eigen::Vector2f target_eigen(linear_function.target.x(), linear_function.target.y());
    Eigen::Vector2f robot_function(linear_function.robot.x(), linear_function.robot.y());
    if((target_eigen - robot_eigen).norm() < (target_eigen - robot_function).norm())
    {
        linear_function.robot = QPointF(r_state.x, r_state.y);
        return true;
    }

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
// this->laser_proxy->getLaserAndr_stateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

