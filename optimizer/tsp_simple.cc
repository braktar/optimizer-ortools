// Copyright © Mapotempo, 2013-2015
//
// This file is part of Mapotempo.
//
// Mapotempo is free software. You can redistribute it and/or
// modify since you respect the terms of the GNU Affero General
// Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Mapotempo is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE.  See the Licenses for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with Mapotempo. If not, see:
// <http://www.gnu.org/licenses/agpl.html>
//
#include <iostream>

#include "base/commandlineflags.h"
#include "constraint_solver/routing.h"
#include "base/join.h"
#include "base/timer.h"
#include <base/callback.h>

#include "tsptw_data_dt.h"
#include "tsptw_solution_dt.h"
#include "routing_common/routing_common_flags.h"

DEFINE_int64(time_limit_in_ms, 2000, "Time limit in ms, 0 means no limit.");
DEFINE_int64(soft_upper_bound, 3, "Soft upper bound multiplicator, 0 means hard limit.");
DEFINE_int64(penalty_cost, 1000000, "Force constraints");
DEFINE_int64(linear_penalty_cost, 50, "Force Constraints");

const char* kCapacity = "capacity";
const char* kTime = "time";
const char* kDistance = "distance";

namespace operations_research {

void TSPTWSolver(const TSPTWDataDT & data) {

  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();
  const int size_tws = data.SizeTimeWindows();

  // Définition des départs et arrivées par véhicule
  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(data.SizeVehicle());
  for( int i = 0; i < data.SizeVehicle(); ++i ) {
    (*start_ends)[i] = std::make_pair(data.VehicleStart(i), data.VehicleEnd(i));
  }
  RoutingModel routing(size, data.SizeVehicle(), *start_ends);
  routing.SetCost(NewPermanentCallback(&data, &TSPTWDataDT::Distance));


  const int64 kNullCapacitySlack = 0;
  const int64 horizon = data.Horizon();

  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::TimePlusServiceTime),
    horizon, horizon, true, kTime);

  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Distance),
    kNullCapacitySlack, std::numeric_limits<int>::max(), true, kDistance);

  if(data.SizeCapacities() > 0){
    routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Demand0),
      kNullCapacitySlack, data.MaxVehicleCapacity(0), true, kCapacity+std::to_string(0));
  }
  if(data.SizeCapacities() > 1){
    routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Demand1),
      kNullCapacitySlack, data.MaxVehicleCapacity(1), true, kCapacity+std::to_string(1));
  }
  if(data.SizeCapacities() > 2){
    routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Demand2),
      kNullCapacitySlack, data.MaxVehicleCapacity(2), true, kCapacity+std::to_string(2));
  }


  routing.GetMutableDimension(kTime)->SetSpanCostCoefficientForAllVehicles(5);
  routing.GetMutableDimension(kTime)->SetEndCumulVarSoftUpperBound(0, horizon, 10000000);

  // Setting Vehicles
  for(int route = 0; route < data.SizeVehicle(); ++route){
    routing.SetFixedCostOfVehicle(data.FixedCostVehicle(route),route);

    //routing.AddVariableMinimizedByFinalizer(routing.GetMutableDimension(kTime)->CumulVar(route));
    //routing.GetMutableDimension(kTime)->CumulVar(routing.Start(route))->SetMin(data.ReadyTimeVehicle(route));
    //routing.GetMutableDimension(kTime)->CumulVar(routing.End(route))->SetMax(data.FinishTimeVehicle(route));

    routing.GetMutableDimension(kTime)->SetStartCumulVarSoftLowerBound(route, data.ReadyTimeVehicle(route),FLAGS_linear_penalty_cost);
    routing.GetMutableDimension(kTime)->SetEndCumulVarSoftUpperBound(route, data.FinishTimeVehicle(route),FLAGS_linear_penalty_cost);

    //routing.GetMutableDimension(kTime)->SetSpanUpperBoundForVehicle(data.WorkEndVehicle(route),route);

    // Objective : Minimizing cost of distance and time
    routing.GetMutableDimension(kDistance)->SetSpanCostCoefficientForVehicle(data.VehicleDistanceCost(route), route);
    routing.GetMutableDimension(kTime)->SetSpanCostCoefficientForVehicle(data.VehicleTimeCost(route), route);

    if(data.SizeCapacities() > 0)
      routing.GetMutableDimension(kCapacity+std::to_string(0))->SetEndCumulVarSoftUpperBound(route, data.Capacity0Vehicle(route), FLAGS_penalty_cost);
    if(data.SizeCapacities() > 1)
      routing.GetMutableDimension(kCapacity+std::to_string(1))->SetEndCumulVarSoftUpperBound(route, data.Capacity1Vehicle(route), FLAGS_penalty_cost);
    if(data.SizeCapacities() > 2)
      routing.GetMutableDimension(kCapacity+std::to_string(2))->SetEndCumulVarSoftUpperBound(route, data.Capacity2Vehicle(route), FLAGS_penalty_cost);
    /*if( FLAGS_has_switch_skills && route%2 == 0 )
      data.SwitchSkills(route);*/
  }

  //  Setting time windows
  for (RoutingModel::NodeIndex i(1); i < size_tws -1 ; ++i) {
    int64 index = routing.NodeToIndex(i);
    IntVar* const cumul_var = routing.CumulVar(index, kTime);
    int64 const ready = data.ReadyTime(i);
    int64 const due = data.DueTime(i);

    if (ready <= 0 && due <= 0) {
      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
      routing.AddDisjunction(*vect, 0); // skip node for free
      cumul_var->SetMin(0);
      cumul_var->SetMax(0);
    } else if (ready > 0 || due > 0) {
      if (ready > 0) {
        cumul_var->SetMin(ready);
      }
      if (due > 0 && due < 2147483647) {
        if (FLAGS_soft_upper_bound > 0) {
          routing.SetCumulVarSoftUpperBound(i, kTime, due, FLAGS_soft_upper_bound);
        } else {
          routing.SetCumulVarSoftUpperBound(i, kTime, due, FLAGS_penalty_cost);
        }
      }
    }
  }

  for(int j = 1; j < data.SizeMatrix()-1 ; ++j){
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(data.NumberTimeWindows(j));
    vect = data.VectorNode(j);
    routing.AddDisjunction(*vect, FLAGS_penalty_cost);
  }

  /*for (int n = 0; n < size_rest; ++n) {
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
    RoutingModel::NodeIndex rest(size_tws + n);
    (*vect)[0] = rest;
    int64 index = routing.NodeToIndex(rest);
    IntVar* const cumul_var = routing.CumulVar(index, kTime);
    int64 const ready = data.ReadyTime(rest);
    int64 const due = data.DueTime(rest);

    if (ready > 0) {
      cumul_var->SetMin(ready);
    }
    if (due > 0 && due < 2147483647) {
      if (FLAGS_soft_upper_bound > 0) {
        routing.SetCumulVarSoftUpperBound(rest, kTime, due, FLAGS_soft_upper_bound);
      } else {
        routing.SetCumulVarSoftUpperBound(rest, kTime, due, FLAGS_penalty_cost);
      }
    }
    routing.AddDisjunction(*vect,FLAGS_penalty_cost);
  }*/

  //  Search strategy
  // routing.set_first_solution_strategy(RoutingModel::ROUTING_DEFAULT_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_GLOBAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_LOCAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_PATH_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_EVALUATOR_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_ALL_UNPERFORMED);
  routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_BEST_INSERTION);

  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GREEDY_DESCENT);
  routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GUIDED_LOCAL_SEARCH);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_SIMULATED_ANNEALING);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_TABU_SEARCH);

  // routing.SetCommandLineOption("routing_no_lns", "true");

  if (FLAGS_time_limit_in_ms > 0) {
    routing.UpdateTimeLimit(FLAGS_time_limit_in_ms);
  }

  Solver *solver = routing.solver();

  std::vector<std::vector<IntVar*> > tBreak;
  for (int n = 0; n < size_rest; ++n) {
    RoutingModel::NodeIndex rest(size_tws+n);
    int64 const ready = data.ReadyTime(rest);
    int64 const due = data.DueTime(rest);
    int64 const service = data.ServiceTime(rest);
    // Setting Rest
    std::vector<IntVar*> isBreak;

    for (RoutingModel::NodeIndex i(0); i < size_tws -1; ++i){
      int64 index = routing.NodeToIndex(i);
      IntVar* const var = solver->MakeBoolVar("break"+ std::to_string(index) );
      routing.AddToAssignment(var);
      solver->AddConstraint(solver->MakeLessOrEqual(var, routing.ActiveVar(index) ));
      solver->AddConstraint(solver->MakeGreaterOrEqual(routing.CumulVar(index,kTime), solver->MakeProd(var, ready)));
      solver->AddConstraint(solver->MakeLessOrEqual(routing.CumulVar(index,kTime), solver->MakeSum(solver->MakeProd(var, due), solver->MakeProd(solver->MakeDifference(1,var),214748364))));
      solver->AddConstraint(solver->MakeGreaterOrEqual(routing.SlackVar(index,kTime), solver->MakeProd(var,service)));
      isBreak.push_back(var);
    }
    solver->AddConstraint(solver->MakeEquality(solver->MakeSum(isBreak),1));
    tBreak.push_back(isBreak);
  }

  const Assignment* solution = routing.Solve(NULL);
  if (solution != NULL) {
    std::cout << "Cost: " << solution->ObjectiveValue() << std::endl;
    TSPTWSolution sol(data, &routing, solution);
    for(int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        std::cout << nodeIndex << " ";
        for (int n = 0; n < size_rest; ++n) {
          int64 v = solution->Value(tBreak.at(n).at(index));
          if(v == 1){
            std::cout << size_tws + n << " ";
          }
        }
      }
      std::cout << routing.IndexToNode(routing.End(route_nbr)) << std::endl;
    }
  } else {
    std::cout << "No solution found..." << std::endl;
  }
}

} // namespace operations_research

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
  operations_research::TSPTWSolver(tsptw_data);

  return 0;
}
