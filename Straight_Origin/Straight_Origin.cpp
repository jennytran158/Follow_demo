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

#include <my-scrimmage-plugins/plugins/autonomy/Straight_Origin/Straight_Origin.h>

#include <scrimmage/common/RTree.h>
#include <scrimmage/math/Angles.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::list;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::Straight_Origin,
                Straight_Origin_plugin)

namespace scrimmage {
namespace autonomy {
  void Straight_Origin::init(std::map<std::string, std::string> &params) {
      initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
      auto callback = [&] (scrimmage::MessagePtr<std::tuple<int,int,Eigen::Vector3d>> msg) {
        auto temp = msg->data;
        int i = std::get<0>(temp);
        int follow_id = std::get<1>(temp);
        Eigen::Vector3d pos = std::get<2>(temp);
        if (!attacking_  && i == parent_->id().id()){
          desired_pos = pos;
          follow_id_ = follow_id;
          attack_time = 0;
          attack_end_time = 0;
        }
      };
      subscribe<std::tuple<int,int,Eigen::Vector3d>>("GlobalNetwork", "paths", callback);
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


      fov_el_ = Angles::deg2rad(get("fov_el", params, 90));
      fov_az_ = Angles::deg2rad(get("fov_az", params, 90));
      comms_range_ = get("comms_range", params, 1000);

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
      }
  }

  bool Straight_Origin::step_autonomy(double t, double dt) {
    // goal_  = Eigen::Vector3d(goal_(0),goal_(1),state_->pos()(2));
    // Find neighbors that are within field-of-view and within comms range
    std::vector<ID> rtree_neighbors;
    rtree_->neighbors_in_range(state_->pos(), rtree_neighbors, comms_range_);
    // Remove neighbors that are not within field of view
    for (auto it = rtree_neighbors.begin(); it != rtree_neighbors.end();
         /* no inc */) {

        // Ignore own position / id
        if (it->id() == parent_->id().id()) {
            it = rtree_neighbors.erase(it);
        } else if (state_->InFieldOfView(*(*contacts_)[it->id()].state(), fov_az_, fov_el_)) {
            // The neighbor is "in front"
            ++it;
        } else {
            // The neighbor is "behind." Remove it.
            it = rtree_neighbors.erase(it);
        }
    }

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
    double team_size = 0;
    double variance = 0.04;
    for (ID id : rtree_neighbors) {
        bool is_team = (id.team_id() == parent_->id().team_id());

        StatePtr other_state = (*contacts_)[id.id()].state();

        // Calculate vector pointing from own position to other
        Eigen::Vector3d diff = other_state->pos() - state_->pos();
        double dist = diff.norm();

        // Calculate magnitude of repulsion vector
        double min_range = is_team ? minimum_team_range_ : minimum_nonteam_range_;
        double O_mag = 0;
        if (dist > sphere_of_influence_) {
            O_mag = 0;
        } else if (min_range < dist && dist <= sphere_of_influence_) {
          double mean = pow(dist - 6, 2);
          O_mag = 1e10 *  exp(-mean/(2*variance));
          double tmp = O_mag;
          printf("mag %f\n",tmp);
              // O_mag = (sphere_of_influence_ - dist) /
              //       (sphere_of_influence_ - min_range);
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
    // if (rtree_neighbors.size() > 0) {
    //     centroid = centroid / static_cast<double>(rtree_neighbors.size());
    //     align = align / static_cast<double>(rtree_neighbors.size());
    //     heading /= static_cast<double>(rtree_neighbors.size());
    //     align_vec << cos(heading), sin(heading), 0;
    // }
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
    centroid = Eigen::Vector3d(200,200,100);
    // Apply gains to independent behaviors
    double radius = 10;
    Eigen::Vector3d intersect = centroid + (state_->pos() - centroid).normalized() * radius;
    double centroid_norm = (intersect - state_->pos()).norm();
    variance = 10;
    double mean = pow(centroid_norm, 2);
    double centroid_mag = 1e10 * (1 - exp(-mean/(2*variance)));
    double tmp = centroid_mag;
    desired_state_->pos() = intersect;
    Eigen::Vector3d v_goal_w_gain = v_goal * w_goal_;
    Eigen::Vector3d O_team_vec_w_gain = O_team_vec * w_avoid_team_;
    Eigen::Vector3d O_nonteam_vec_w_gain = O_nonteam_vec * w_avoid_nonteam_;
    Eigen::Vector3d v_centroid_w_gain = (intersect - state_->pos()).normalized() * w_centroid_ * centroid_mag;
    Eigen::Vector3d v_align_w_gain = v_align_normed * w_align_;

    double sum_norms = O_team_vec_w_gain.norm() +
        O_nonteam_vec_w_gain.norm() + v_centroid_w_gain.norm() +
        v_align_norm;

    Eigen::Vector3d v_sum = ( O_team_vec_w_gain +
                             O_nonteam_vec_w_gain + v_centroid_w_gain +
                             v_align_w_gain) / sum_norms;

    // Scale velocity to max speed:
    //sum_norms < norms of sum => v_sum < 1
    Eigen::Vector3d vel_result = v_sum * max_speed_;

    if (rtree_neighbors.size() > 0) {

        velocity_controller(vel_result);

        if (show_shapes_) {
            auto inter = sc::shape::make_sphere(intersect, 2, Eigen::Vector3d(255, 0, 0),0.1);
            inter->set_persistent(false);
            inter->set_ttl(1);
            draw_shape(inter);
            if(parent_->id().id() == 2) {
            auto sphere = sc::shape::make_sphere(centroid, radius, Eigen::Vector3d(0, 255, 0),0.1);
            sphere->set_persistent(false);
            sphere->set_ttl(1);
            draw_shape(sphere);
          }
            // Draw resultant vector:
            auto line = sc::shape::make_line(state_->pos(), vel_result + state_->pos(),Eigen::Vector3d(255, 255, 0),0.75);
            line->set_persistent(false);
            line->set_ttl(1);
            draw_shape(line);
        }
    } else {
      velocity_controller(v_goal);
    }
    return true;
  }
  void Straight_Origin::velocity_controller(Eigen::Vector3d &v) {
      // Convert to spherical coordinates:
      double desired_heading = atan2(v(1), v(0));
      double desired_pitch = atan2(v(2), v.head<2>().norm());

      Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX();
      Eigen::Vector3d temp = state_->pos() + v * rel_pos.norm();
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
} // namespace autonomy
} // namespace scrimmage
