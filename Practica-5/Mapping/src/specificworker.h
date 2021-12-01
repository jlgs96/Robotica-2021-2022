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
#include <grid2d/grid.h>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
//#include <grid2d/grid.cpp>
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();


    ///////ESTRUCTURA PARA DEFINIR EL TARGET DEL ROBOT//////////
    struct Target
    {
        QPointF destiny;
        bool active;
    };

    //////ESTRUCTURA PARA DEFINIR LA FUNCIÓN DE LA RECTA DEL ROBOT/////
    struct LinearFunction
    {
        QPointF robot;
        QPointF target;
    };

    /////ESTRUCTURA PARA DEFINIR LA PUERTA/////
    struct Door
    {
        Eigen::Vector2f p1;
        Eigen::Vector2f p2;
        std::vector<Eigen::Vector2f> midPointDoors;
        int h1, h2;
        bool operator== (Door door){return ((p1-door.p1).norm() < 800 and (p2-door.p2).norm()< 800) or ((p1-door.p2).norm()<800 and (p2-door.p1).norm()<800);}
        Door(Eigen::Vector2f c1,Eigen::Vector2f c2):p1(c1), p2(c2) { };

    };

    //////INICIALIZACION DE LAS ESTRUCTURAS///////////
    Target target;
    LinearFunction linear_function;
    std::vector<Door>Doors;

    /////DECLARACIÓN DEL TIPO ENUMERADO PARA LOS ESTADOS DEL ROBOT/////

    ///OLD ENUM STATES///
    //enum class State {IDLE, RUN, OBSTACLE, SURROUND};
    //State estado = State::IDLE;

    /*
     * NUEVA DECLARACIÓN DE LOS ESTADOS:
     * EPXLORE: MAPPING DE LA HABITACIÓN ACTUAL
     * DOOR: BUSCA LA SALIDA DE LA HABITACIÓN
     * CHANGEROOM: CAMBIA DE UNA HABITACION A OTRA
     * CENTERROOM: SE COLOCA EN EL CENTRO DE LA HABITACIÓN PARA MAPPING
     */
    enum class State {EXPLORE, DOOR, CHANGEROOM, CENTERROOM};
    State mappState = State::EXPLORE;



    ////////////METODOS DE PROPOSITO GENERAL//////////
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    QPointF world_to_robot( const RoboCompFullPoseEstimation::FullPoseEuler r_state);

    ///OLD, MAYBE REUSABLE METODOS DE PROPOSITO GENERAL///
    float dist_to_target_object(float dist);
    float rotation_speed(float beta);
    bool isInFunction(const  RoboCompFullPoseEstimation::FullPoseEuler &r_state);




    ///////////VARIABLES VARIAS/////////////////////////
    float MAX_ADV_SPEED = 1000.0;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF target_to_robot;
    QPolygonF  poly;


    /////VARIABLES MAPPING/////
    const int MAX_LASER_DIST = 4000;
    const int TILE = 100;
    std::vector<Eigen::Vector2f>midPointsDoorGlobal[2];



    ////METODOS MAPPING////
    void update_map(const RoboCompLaser::TLaserData  &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    Eigen::Vector2f robot_to_world( const RoboCompFullPoseEstimation::FullPoseEuler &r_state, Eigen::Vector2f cartesianP);
    State exploringRoom(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    State lookDoor(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    void centerRoom(const RoboCompLaser::TLaserData &ldata);
    void paintDoor(const std::vector<Eigen::Vector2f> &peaks);
    ///////SLOTS DE CONEXION/////
public slots:
	void compute();
	int startup_check();
	void initialize(int period);
void new_target_slot(QPointF point);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    bool distance_ahead(const RoboCompLaser::TLaserData &ldata, float dist, int semiwidth);
    RoboCompLaser::TData get_min_ldata_element(const RoboCompLaser::TLaserData &ldata, int semiwidth);
    bool distance_side(const RoboCompLaser::TLaserData &ldata, float dist, int iterBegin, int iterEnd);
    /////GRID VARIABLE/////
    Grid Mapp;

    void calculate_door_points(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state);

    /////CONSIGUE EL ELEMENTO MAXIMO/////
    RoboCompLaser::TData get_max_ldata_element(const RoboCompLaser::TLaserData &ldata, int semiwidth);

};

#endif
