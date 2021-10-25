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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();


    ///////ESTRUCTURA PARA DEFINIR EL TARGET DEL ROBOT//////////
    typedef struct{
        QPointF destiny;
        bool active;
    }Target;

    //////ESTRUCTURA PARA DEFINIR LA FUNCIÓN DE LA RECTA DEL ROBOT/////
    typedef struct {
        QPointF robot;
        QPointF target;
    }LinearFunction;
    //////INICIALIZACION DE LAS ESTRUCTURAS///////////
    Target target;
    LinearFunction function;

    /////DECLARACIÓN DEL TIPO ENUMERADO PARA LOS ESTADOS DEL ROBOT/////
    enum class State {IDLE, RUN, OBSTACLE, SURROUND};
    State estado = State::IDLE;

    ////////////METODOS DE PROPOSITO GENERAL//////////
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    QPointF world_to_robot(Target target, RoboCompGenericBase::TBaseState bState);
    float dist_to_target(float dist);
    float rotation_speed(float beta);



    ////////METODOS DE COMPORTAMIENTO ROBOT///////////////
    State run(const RoboCompGenericBase::TBaseState &bState, Target &target);
    State obstacle(const RoboCompGenericBase::TBaseState &bState, Target &target);
    State surround(const RoboCompGenericBase::TBaseState &bState, Target &target);

    ///////////VARIABLES VARIAS/////////////////////////
    float MAX_ADV_SPEED = 1000;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF target_to_robot;
    QPolygonF  poly;
    ///////SLOTS DE CONEXION/////
public slots:
	void compute();
	int startup_check();
	void initialize(int period);
void new_target_slot(QPointF point);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

};

#endif
