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

#include <my-scrimmage-plugins/plugins/autonomy/Pure_Pursuit/Pure_Pursuit.h>
#include <scrimmage/common/RTree.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>

#include <scrimmage/parse/ParseUtils.h>
#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Pure_Pursuit,
  Pure_Pursuit_plugin)

  namespace scrimmage {
    namespace autonomy {
      void Pure_Pursuit::init(std::map<std::string, std::string> &params) {
        initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
        killed_ids_ = advertise("GlobalNetwork", "Killed_IDs",100);
        auto killed_callback = [&] (scrimmage::MessagePtr<int> msg) {
          if (msg->data == follow_id_) {
            attack_time = 0;
            attack_end_time = 0;
            follow_id_ = -1;
          }
        };
        subscribe<int>("GlobalNetwork", "Killed_IDs", killed_callback);

        show_shapes_ = get("show_shapes", params, true);
        max_speed_ = get<double>("max_speed", params, 21);

        w_align_ = get("align_weight", params, 0.01);
        w_avoid_team_ = get("avoid_team_weight", params, 0.95);
        w_centroid_ = get("centroid_weight", params, 0.05);

        w_avoid_nonteam_ = get("avoid_nonteam_weight", params, 1.0);

        fov_el_ = M_PI*get("fov_el", params, 90)/180;
        fov_az_ = M_PI*get("fov_az", params, 90)/180;
        // comms_range_ = get("comms_range", params, 1000);

        fov_height_angle_ = M_PI*get("fov_height_angle", params, 90)/180;
        fov_width_angle_ = M_PI*get("fov_width_angle", params, 90)/180;
        view_length_ = get("view_length", params, 1000);

        attack_fov_height_angle_ = M_PI*get("attack_fov_height_angle", params, 90)/180;
        attack_fov_width_angle_ = M_PI*get("attack_fov_width_angle", params, 90)/180;
        attack_view_length_ = get("attack_view_length", params, 1000);

        sphere_of_influence_ = get<double>("sphere_of_influence", params, 10);
        minimum_team_range_ = get<double>("minimum_team_range", params, 5);
        minimum_nonteam_range_ = get<double>("minimum_nonteam_range", params, 10);

        w_goal_ = get<double>("goal_weight", params, 1.0);

        if (get("use_initial_heading", params, false)) {
          Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000000;
          Eigen::Vector3d unit_vector = rel_pos.normalized();
          printf("unit_vector: %lf %lf %lf\n",unit_vector(0),unit_vector(1),unit_vector(2));
          unit_vector = state_->quat().rotate(unit_vector);
          goal_ = state_->pos() + unit_vector * rel_pos.norm();
        } else {
          std::vector<double> goal_vec;
          if (get_vec<double>("goal", params, " ", goal_vec, 3)) {
            goal_ = vec2eigen(goal_vec);
          }
          goal_ = state_->pos() ;

        }



        io_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
        io_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
        io_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

        io_vel_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
        io_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
        io_pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);

        io_desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
        io_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
        io_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
      }

      bool Pure_Pursuit::step_autonomy(double t, double dt) {
        // Find neighbors that are within field-of-view and within comms range
        std::vector<ID> rtree_neighbors;
        rtree_->neighbors_in_range(state_->pos(), rtree_neighbors, view_length_);


        // Remove neighbors that are not within field of view
        bool following_in_view = false;
        for (auto it = rtree_neighbors.begin(); it != rtree_neighbors.end();
      /* no inc */) {

      // Ignore own position / id
      if (it->id() == parent_->id().id()) {
        it = rtree_neighbors.erase(it);
      } else {
        auto other = (*contacts_)[it->id()];
        sc::StatePtr other_state = other.state();
        auto other_pos = other_state->pos();
        //if is_teammate and in communication range
        if (other.id().team_id() == parent_->id().team_id() && other_pos != Eigen::Vector3d(0,0,0) &&
        state_->InFieldOfView(*other_state, fov_az_, fov_el_)) {
          ++it;

          //if enemy and in vision range
        } else if (other.id().team_id() != parent_->id().team_id() &&
        state_->InFieldOfView(*other_state, fov_width_angle_, fov_height_angle_)) {
          if (it->id() == follow_id_) {
            following_in_view = true;
          }
          ++it;
        } else {
          // The enemy is "behind." Remove it.
          it = rtree_neighbors.erase(it);
        }
      }
    }
    // __FOV
    // double fov_width = 2 * (view_length_ / tan((M_PI - fov_width_angle_)/2));
    // double fov_height = 2 * (view_length_ / tan((M_PI - fov_height_angle_)/2));
    // printf("fov_width_angle_: %lf, view_length_: %lf\n",fov_width_angle_, view_length_);
    // std::list<Eigen::Vector3d> view_points;
    // std::list<Eigen::Vector3d> view_points_transformed;
    // view_points.push_back(Eigen::Vector3d(0,0,0));
    // view_points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
    // view_points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2, fov_height/2));
    // view_points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2,  -fov_height/2));
    // view_points.push_back(Eigen::Vector3d(-view_length_, fov_width/2, -fov_height/2));
    // view_points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
    // view_points.push_back(Eigen::Vector3d(0,0,0));
    // for (auto iter = view_points.begin(); iter != view_points.end(); iter++) {
    //   view_points_transformed.push_back(state_->pos() - (state_->quat().normalized() * *iter));
    // }
    // auto FOV = sc::shape::make_polygon(view_points_transformed, Eigen::Vector3d(0, 0, 0),0.1);
    // FOV->set_persistent(false);
    // FOV->set_ttl(1);
    // draw_shape(FOV);
    if (!following_in_view){
      follow_id_ = -1;
    }
    if (follow_id_ > 0) {
      StatePtr enemy_state = (*contacts_)[follow_id_].state();
      try {
        goal_ = enemy_state->pos();
      } catch (const std::bad_typeid &e) {
        follow_id_ = -1;
      }
    }
    if (follow_id_ < 0) {
      goal_ = Eigen::Vector3d(0,0,100);
    }
    // printf("goal_: %lf %lf %lf, hasNaN: %s\n",goal_(0),goal_(1),goal_(2), goal_.hasNaN() ? "true" : "false");

    // move-to-goal behavior
    Eigen::Vector3d v_goal = (goal_ - state_->pos()).normalized();

    // Steer to avoid local neighbors
    // Align with neighbors
    // Cohesion: move towards average position of neighbors
    // (i.e., Find centroid of neighbors)
    double heading = 0;
    Eigen::Vector3d align(0, 0, 0);
    Eigen::Vector3d centroid(0, 0, 0);

    // Compute repulsion vector from each robot (team/nonteam)
    std::vector<Eigen::Vector3d> O_team_vecs;
    std::vector<Eigen::Vector3d> O_nonteam_vecs;
    double min_dist = 1e10;
    double team_size = 0;
    for (ID id : rtree_neighbors) {
      bool is_team = (id.team_id() == parent_->id().team_id());

      StatePtr other_state = (*contacts_)[id.id()].state();
      auto other_pos = other_state->pos();

      // Calculate vector pointing from own position to other
      Eigen::Vector3d diff = other_pos - state_->pos();
      double dist = diff.norm();
      if (!is_team && dist < min_dist ) {
        follow_id_ = id.id();
        min_dist = dist;
      }
      // Calculate magnitude of repulsion vector
      double min_range = is_team ? minimum_team_range_ : minimum_nonteam_range_;
      double O_mag = 0;
      if (dist > sphere_of_influence_) {
        O_mag = 0;
      } else if (min_range < dist && dist <= sphere_of_influence_) {
        O_mag = (sphere_of_influence_ - dist) /
        (sphere_of_influence_ - min_range);
      } else if (dist <= min_range) {
        O_mag = 1e10;
      }

      // Calculate repulsion vector
      Eigen::Vector3d O_dir = - O_mag * diff.normalized();
      if (is_team) {
        O_team_vecs.push_back(O_dir);
      } else {
        O_nonteam_vecs.push_back(O_dir);
      }

      // Calculate centroid of team members and heading alignment
      if (is_team) {
        centroid = centroid + other_state->pos();
        align += other_state->vel().normalized();
        heading += other_state->quat().yaw();
        team_size += 1;
      }
    }

    Eigen::Vector3d align_vec(0, 0, 0);
    if (team_size > 0) {
      centroid = centroid / team_size;
      align = align / team_size;
      heading /= team_size;
      align_vec << cos(heading), sin(heading), 0;
    }

    // Make sure alignment vector is well-behaved
    align = align_vec; // TODO
    Eigen::Vector3d v_align_normed = align.normalized();
    double v_align_norm = align.norm();
    if (v_align_normed.hasNaN()) {
      v_align_normed = Eigen::Vector3d::Zero();
      v_align_norm = 0;
    }

    // Normalize each team repulsion vector and sum
    Eigen::Vector3d O_team_vec(0, 0, 0);
    for (Eigen::Vector3d v : O_team_vecs) {
      if (v.hasNaN()) {
        continue; // ignore misbehaved vectors
      }
      O_team_vec += v;
    }

    // Normalize each nonteam repulsion vector and sum
    Eigen::Vector3d O_nonteam_vec(0, 0, 0);
    for (Eigen::Vector3d v : O_nonteam_vecs) {
      if (v.hasNaN()) {
        continue; // ignore misbehaved vectors
      }
      O_nonteam_vec += v;
    }

    // Apply gains to independent behaviors
    Eigen::Vector3d v_goal_w_gain = v_goal * w_goal_;
    Eigen::Vector3d O_team_vec_w_gain = O_team_vec * w_avoid_team_;
    Eigen::Vector3d O_nonteam_vec_w_gain = O_nonteam_vec * w_avoid_nonteam_;
    Eigen::Vector3d v_centroid_w_gain = (centroid - state_->pos()).normalized() * w_centroid_;
    Eigen::Vector3d v_align_w_gain = v_align_normed * w_align_;
    // printf("v_goal_w_gain: %lf %lf %lf, hasNaN: %s\n",v_goal_w_gain(0),v_goal_w_gain(1),v_goal_w_gain(2), v_goal_w_gain.hasNaN() ? "true" : "false");
    // printf("O_team_vec_w_gain: %lf %lf %lf, hasNaN: %s\n",O_team_vec_w_gain(0),O_team_vec_w_gain(1),O_team_vec_w_gain(2), O_team_vec_w_gain.hasNaN() ? "true" : "false");
    // printf("O_nonteam_vec_w_gain: %lf %lf %lf, hasNaN: %s\n",O_nonteam_vec_w_gain(0),O_nonteam_vec_w_gain(1),O_nonteam_vec_w_gain(2), O_nonteam_vec_w_gain.hasNaN() ? "true" : "false");
    // printf("v_centroid_w_gain: %lf %lf %lf, hasNaN: %s\n",v_centroid_w_gain(0),v_centroid_w_gain(1),v_centroid_w_gain(2), v_centroid_w_gain.hasNaN() ? "true" : "false");
    // printf("v_align_w_gain: %lf %lf %lf, hasNaN: %s\n",v_align_w_gain(0),v_align_w_gain(1),v_align_w_gain(2), v_align_w_gain.hasNaN() ? "true" : "false");

    double sum_norms = v_goal_w_gain.norm() + O_team_vec_w_gain.norm() +
    O_nonteam_vec_w_gain.norm() + v_centroid_w_gain.norm() +
    v_align_norm;
    Eigen::Vector3d v_sum = (v_goal_w_gain + O_team_vec_w_gain +
      O_nonteam_vec_w_gain + v_centroid_w_gain +
      v_align_w_gain) / sum_norms;

      // Scale velocity to max speed:
      Eigen::Vector3d vel_result = v_sum * max_speed_;
      // printf("v_sum: %lf %lf %lf, hasNaN: %s\n",v_sum(0),v_sum(1),v_sum(2), v_sum.hasNaN() ? "true" : "false");

      if (rtree_neighbors.size() > 0) {

        velocity_controller(vel_result);

        if (show_shapes_) {

          // Draw resultant velocity vector:
          auto line = sc::shape::make_line(state_->pos(), vel_result + state_->pos(),Eigen::Vector3d(255, 255, 0),0.75);
          line->set_persistent(false);
          line->set_ttl(1);
          draw_shape(line);
        }
      } else {
        velocity_controller(v_goal);
      }
      if (goal_ != Eigen::Vector3d(0,0,100)) {
        check_attack(t, dt);
      }


      //animate attack field of view
      // double attack_fov_width = 2 * (attack_view_length_ / tan((M_PI - attack_fov_width_angle_)/2));
      // double attack_fov_height = 2 * (attack_view_length_ / tan((M_PI - attack_fov_height_angle_)/2));
      // std::list<Eigen::Vector3d> attack_view_points;
      // std::list<Eigen::Vector3d> attack_view_points_transformed;
      // attack_view_points.push_back(Eigen::Vector3d(0,0,0));
      // attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
      // attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2, attack_fov_height/2));
      // attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2,  -attack_fov_height/2));
      // attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, attack_fov_width/2, -attack_fov_height/2));
      // attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
      // attack_view_points.push_back(Eigen::Vector3d(0,0,0));
      // for (auto iter = attack_view_points.begin(); iter != attack_view_points.end();iter++){
      //   attack_view_points_transformed.push_back(state_->pos() - (state_->quat().normalized() * *iter));
      // }
      // auto attack_FOV = sc::shape::make_polygon(attack_view_points_transformed, Eigen::Vector3d(255, 0, 0),0.1);
      // attack_FOV->set_persistent(false);
      // attack_FOV->set_ttl(1);
      // draw_shape(attack_FOV);
      return true;
    }
    void Pure_Pursuit::velocity_controller(Eigen::Vector3d &v) {
      // Convert to spherical coordinates:
      double desired_heading = atan2(v(1), v(0));
      double desired_pitch = atan2(v(2), v.head<2>().norm());

      Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000;
      Eigen::Vector3d temp = state_->pos() + v * rel_pos.norm();
      desired_state_->pos() = Eigen::Vector3d(temp(0),temp(1),state_->pos()(2) + v(2));
      desired_state_->quat().set(0, 0,desired_heading);

      // vars_.output(io_vel_idx_, max_speed_);
      // vars_.output(io_turn_rate_idx_, desired_heading - state_->quat().yaw());
      // vars_.output(io_pitch_rate_idx_, desired_pitch + state_->quat().pitch());
      //
      // vars_.output(io_heading_idx_, desired_heading);
      // vars_.output(io_altitude_idx_, v(2));
      // vars_.output(io_desired_speed_idx_, max_speed_);

      double norm = v.norm();
      double ratio = (max_speed_ / 2) / std::max(norm, 1.0);
      if (norm != 0 && ratio < 1) {
        v *= ratio;
      }
    }
    void Pure_Pursuit::check_attack(double t, double dt) {
      int id = follow_id_;

      Eigen::Vector3d &pos = state_->pos();
      auto ent = contacts_->at(id);
      follow_id_ = id;
      sc::StatePtr ent_state = ent.state();
      Eigen::Vector3d &ent_pos = ent_state->pos();
      Eigen::Vector2d p = (ent_pos - pos).head<2>();
      double dist = p.norm();

      // Red line: indicate a  pair of drone and its target
      auto line = sc::shape::make_line(state_->pos(), goal_,Eigen::Vector3d(255, 0, 0),0.75);
      line->set_persistent(false);
      line->set_ttl(1);
      draw_shape(line);

      if (dist <= attack_view_length_ &&
        state_->InFieldOfView(*ent_state, attack_fov_width_angle_, attack_fov_height_angle_)) {
          attacking_ = true;
          if (attack_time == 0){
            attack_time = t;
            attack_end_time = attack_time + dt * attack_duration;
          }
          if (attack_time >= attack_end_time){
            attack_time = 0;
            attack_end_time = 0;
            //broadcast attacked ids
            auto msg = std::make_shared<sc::Message<int>>();
            msg->data = follow_id_;
            killed_ids_->publish(msg);
          }

          if (attack_time == t) {
            attack_time = t + dt;
          } else {
            attack_time = 0;
          }

          //animate attack field of view
          Eigen::Vector3d attack_color = Eigen::Vector3d(100, 0, 0);
          double attack_fov_width = 2 * (attack_view_length_ / tan((M_PI - attack_fov_width_angle_)/2));
          double attack_fov_height = 2 * (attack_view_length_ / tan((M_PI - attack_fov_height_angle_)/2));
          std::list<Eigen::Vector3d> attack_view_points;
          std::list<Eigen::Vector3d> attack_view_points_transformed;
          attack_view_points.push_back(Eigen::Vector3d(0,0,0));
          attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
          attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2, attack_fov_height/2));
          attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2,  -attack_fov_height/2));
          attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, attack_fov_width/2, -attack_fov_height/2));
          attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
          attack_view_points.push_back(Eigen::Vector3d(0,0,0));
          for (auto iter = attack_view_points.begin(); iter != attack_view_points.end();iter++){
            attack_view_points_transformed.push_back(pos - (state_->quat().normalized() * *iter));
          }
          auto attack_FOV = sc::shape::make_polygon(attack_view_points_transformed, Eigen::Vector3d(255, 0, 0),0.1);
          attack_FOV->set_persistent(false);
          attack_FOV->set_ttl(1);
          draw_shape(attack_FOV);

          double fov_width = 2 * (view_length_ / tan((M_PI - fov_width_angle_)/2));
          double fov_height = 2 * (view_length_ / tan((M_PI - fov_height_angle_)/2));
          std::list<Eigen::Vector3d> view_points;
          std::list<Eigen::Vector3d> view_points_transformed;
          view_points.push_back(Eigen::Vector3d(0,0,0));
          view_points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
          view_points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2, fov_height/2));
          view_points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2,  -fov_height/2));
          view_points.push_back(Eigen::Vector3d(-view_length_, fov_width/2, -fov_height/2));
          view_points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
          view_points.push_back(Eigen::Vector3d(0,0,0));
        }
      }
    } // namespace autonomy
  } // namespace scrimmage
