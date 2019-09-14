/*!
* @file
*
* @section LICENSE
*
* Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
*
* This file is part of SCRIMMAGE.
*
*   SCRIMMAGE is free software: you can redistribute it and/or modify it under
*   the terms of the GNU Lesser General Public License as published by the
*   Free Software Foundation, either version 3 of the License, or (at your
*   option) any later version.
*
*   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
*   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
*   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
*   License for more details.
*
*   You should have received a copy of the GNU Lesser General Public License
*   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
*
* @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
* @author Eric Squires <eric.squires@gtri.gatech.edu>
* @date 31 July 2017
* @version 0.1.0
* @brief Brief file description.
* @section DESCRIPTION
* A Long description goes here.
*
*/

#include <my-scrimmage-plugins/plugins/autonomy/Base/Base.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <vector>
#include <list>
using namespace std;
#include <iostream>
#include <limits>
// #include <scrimmage/common/Shape.h>
using std::cout;
using std::endl;
using std::vector;
#include <voro++/voro++.hh>
using namespace voro;
#include <scrimmage/common/Shape.h>
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()
namespace sc = scrimmage;
#include "boost/polygon/voronoi.hpp"
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Base,
  Base_plugin)


  struct Segment {
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
  };
  namespace boost {
    namespace polygon {

      template <>
      struct geometry_concept<Eigen::Vector3d> {
        typedef point_concept type;
      };

      template <>
      struct point_traits<Eigen::Vector3d> {
        typedef int coordinate_type;

        static inline coordinate_type get(
          const Eigen::Vector3d &point, orientation_2d orient) {
            return (orient == HORIZONTAL) ? point(0) : point(1);
          }
        };

        template <>
        struct geometry_concept<Segment> {
          typedef segment_concept type;
        };

        template <>
        struct segment_traits<Segment> {
          typedef int coordinate_type;
          typedef Eigen::Vector3d point_type;

          static inline point_type get(const Segment& segment, direction_1d dir) {
            return dir.to_int() ? segment.p1 : segment.p0;
          }
        };
      }  // polygon
    }  // boost
    namespace scrimmage {
      namespace autonomy {
        Eigen::Vector3d intersectPoint(Eigen::Vector3d lineVector, Eigen::Vector3d linePoint, Eigen::Vector3d planeNormal, Eigen::Vector3d planePoint) {
          Eigen::Vector3d diff = linePoint - planePoint;
          double prod1 = diff.dot(planeNormal);
          double prod2 = lineVector.dot(planeNormal);
          double prod3 = prod1 / prod2;
          return linePoint - lineVector * prod3;
        }

        bool within_boundary(Eigen::Vector3d pos, Eigen::Vector3d boundary) {
          bool ret = (std::abs(pos(0)) <= boundary(0) & std::abs(pos(1)) <= boundary(1));
          return ret ;
        }
        Eigen::Vector3d find_centroid(list<Eigen::Vector3d> points) {
          double total_area = 0;
          Eigen::Vector3d centroid = Eigen::Vector3d(0,0,0);
          auto iter = points.begin();
          //triangulate polygon to find centroid
          Eigen::Vector3d v = *iter;
          iter++;
          Eigen::Vector3d w = *iter;
          iter++;
          for (; iter != points.end();iter++){
            auto new_point = *iter;
            auto cp = (new_point - v).cross((new_point - w));
            auto area = cp.norm() / 2;
            centroid = centroid + ((area * 1/3) * Eigen::Vector3d(v(0) + w(0) + new_point(0),v(1) + w(1) + new_point(1),v(2) + w(2) + new_point(2)));
            total_area = total_area + area;
            // printf("v:%lf %lf %lf\n",v(0),v(1),v(2));
            // printf("w:%lf %lf %lf\n",w(0),w(1),w(2));
            // printf("new_point:%lf %lf %lf\n",new_point(0),new_point(1),new_point(2));
            // printf("area:%lf\n",area);
            w = new_point;
          }
          // printf("%lf %lf %lf, total_area: %lf\n",centroid(0),centroid(1),centroid(2), total_area);
          centroid = (1/total_area)  * Eigen::Vector3d(centroid(0), centroid(1), centroid(2));
          // printf("new centroid: %lf %lf %lf\n", centroid(0),centroid(1),centroid(2));
          return centroid;
        }
        void Base::init(std::map<std::string, std::string> &params) {
          auto base = std::make_shared<scrimmage_proto::Shape>();
          base->set_opacity(0.25);
          base->set_persistent(true);
          sc::set(base->mutable_color(), 0, 255, 0); // r, g, b
          //our base measurements
          base_width = 50;
          base_height = 50;
          base_depth =  200;
          base->mutable_cuboid()->set_x_length(base_width);
          base->mutable_cuboid()->set_y_length(base_height);
          base->mutable_cuboid()->set_z_length(base_depth);
          sc::set(base->mutable_cuboid()->mutable_center(), 0, 0, base_depth/2);
          draw_shape(base);

          //protected area's boundary
          auto boundary = std::make_shared<scrimmage_proto::Shape>();
          boundary->set_opacity(0.15);
          boundary->set_persistent(true);
          boundary_width = 400;
          boundary_height = 400;
          boundary_depth = 400;
          sc::set(boundary->mutable_color(), 0, 0, 255); // r, g, b
          boundary->mutable_cuboid()->set_x_length(boundary_width);
          boundary->mutable_cuboid()->set_y_length(boundary_height);
          boundary->mutable_cuboid()->set_z_length(boundary_depth);
          sc::set(boundary->mutable_cuboid()->mutable_center(), 0, 0, boundary_depth/2);
          draw_shape(boundary);
          next_paths_ = advertise("GlobalNetwork", "Attacking",10);
          auto callback = [&] (scrimmage::MessagePtr<int> msg) {
            // found_enemy_ids_.push_back(msg->data);
          };
          subscribe<int>("GlobalNetwork", "Enemy_IDs", callback);
          auto killed_callback = [&] (scrimmage::MessagePtr<int> msg) {
            killed_ids_.push_back(msg->data);
          };
          subscribe<int>("GlobalNetwork", "Killed_IDs", killed_callback);
        }

        bool Base::step_autonomy(double t, double dt) {

            // for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
            //   if (it->second.id().team_id() != parent_->id().team_id()) {
            //       auto enemy_pos = it->second.state()->pos();
            //
            //   }
            // }
          // container con(-boundary_width/2,boundary_width/2,-boundary_height/2,boundary_height/2,0,boundary_depth,5,5,5,
          //   false,false,false,1);
          //   particle_order enemies;
          //   double x,y,z;
          //   int id,i,j;
          //   voronoicell_neighbor c;
          //   vector<int> neigh,f_vert;
          //   vector<double> v;
          //   //add a voronoi cell for each found enemy's position and our aircraft's position
          //   // printf("\n___________\n");
          //   std::vector<int> enemy_clusters;
          //   for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
          //     Eigen::Vector3d &pos = it->second.state()->pos();
          //     if (it->second.id().id() != parent_->id().id()) {
          //       // printf("base id: %d\n",it->second.id().id());
          //       if (!(it->second.id().team_id() == parent_->id().team_id())) {
          //         auto enemy_id = it->second.id().id();
          //         auto enemy_pos = it->second.state()->pos();
          //         // printf("base enemy_id: %d\n",it->second.id().id());
          //         if (killed_ids_.empty() ||
          //         killed_ids_.end() == std::find(killed_ids_.begin(),killed_ids_.end(), id)) {
          //           bool new_ent = true;
          //           if (!enemy_clusters.empty()) {
          //             for (auto cluster = enemy_clusters.begin(); cluster != enemy_clusters.end(); cluster++) {
          //               auto center_enemy = contacts_->at(*cluster);
          //               Eigen::Vector3d &center_pos = center_enemy.state()->pos();
          //               auto diff = (center_pos - enemy_pos);
          //               if (diff.norm() < 200) {
          //                 new_ent = false;
          //                 break;
          //               }
          //             }
          //           }
          //           if (new_ent) {
          //             enemy_clusters.push_back(enemy_id);
          //             con.put(enemies,enemy_id,pos(0),pos(1),pos(2));
          //           }
          //         }
          //         // }
          //       } else{
          //         con.put(it->second.id().id(),pos(0),pos(1),pos(2));
          //       }
          //     }
          //   }
          //   c_loop_order clo(con,enemies);
          //   if(clo.start()) do if(con.compute_cell(c,clo)) {
          //     clo.pos(x,y,z);
          //     id=clo.pid();
          //     c.neighbors(neigh);
          //     c.face_vertices(f_vert);
          //     c.vertices(x,y,z,v);
          //     for(i=0,j=0;i<neigh.size();i++) {
          //       if(neigh[i]>id || neigh[i]<0) {
          //         list<Eigen::Vector3d> points;
          //         int k,l,n=f_vert[j];
          //         for(k=0;k<n;k++) {
          //           l = 3*f_vert[j+k+1];
          //           points.push_back(Eigen::Vector3d(v[l],v[l+1],v[l+2]));
          //         }
          //         auto polygon = sc::shape::make_polygon(points,
          //         Eigen::Vector3d(0, 255, 0), 0.35);
          //         polygon->set_persistent(false);
          //         polygon->set_ttl(1);
          //         draw_shape(polygon);
          //         Eigen::Vector3d centroid = find_centroid(points);
          //         auto centroid_sphere = sc::shape::make_sphere(centroid, 1, Eigen::Vector3d(255, 255, 0),1);
          //         centroid_sphere->set_persistent(false);
          //         centroid_sphere->set_ttl(1);
          //         draw_shape(centroid_sphere);
          //         //broadcast next path
          //         if(neigh[i]>0) {
          //           auto msg = std::make_shared<sc::Message<std::tuple<int,int,Eigen::Vector3d>>>();
          //           msg->data = std::make_tuple(neigh[i],id,Eigen::Vector3d(centroid(0),centroid(1),centroid(2)));
          //           next_paths_->publish(msg);
          //         }
          //       }
          //       j+=f_vert[j]+1;
          //     }
          //   } while (clo.inc());
          // //reset
          // // printf("\n___________\n");
          // killed_ids_ = {};
          return true;
        }
      } // namespace autonomy
    } // namespace scrimmage
