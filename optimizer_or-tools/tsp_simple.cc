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
DEFINE_string(vehicle_file, "", "File containing all the vehicles infos");
DEFINE_bool(skip_node, false, "Allow to skip some customers from the plan");
DEFINE_int64(back_to_depot, 0, "Times a vehicle could come back reloading at the depot");
DEFINE_int64(fix_cost_skip_node, 5000, "Fixed Cost if a node is skipped");
DEFINE_int64(linear_cost_skip_node, 1000, "linear cost if a node node is skipped depending on the first dimension");

const char* kCapacity = "Capacity";
const char* kTime = "Time";
const char* kDistance = "Distance";

namespace operations_research {

void TSPTWSolver(const TSPTWDataDT & data) {

  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();

  const RoutingModel::NodeIndex kDepot(0);

  // Définition des départs et arrivées par véhicule
  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(1);
  for( int i = 0; i < data.SizeVehicle(); ++i ) {
	(*start_ends)[i] = std::make_pair(data.Start(), data.Stop());
  }

  RoutingModel routing(size, data.SizeVehicle()*(1+FLAGS_back_to_depot),*start_ends);
  //routing.SetDepot(kDepot);

  const int64 kNullCapacitySlack = 0;
  const int64 horizon = data.Horizon();
  const int64 capacity_horizon_0 = data.Capacity_Horizon(0);
  const int64 capacity_horizon_1 = data.Capacity_Horizon(1);

  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Distance),
    horizon, horizon, true, kDistance);
  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::TimePlusServiceTime),
    horizon, horizon, true, kTime);

	if(data.SizeCapacities() > 0)
		routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Demand0),
			kNullCapacitySlack, capacity_horizon_0, true, kCapacity+std::to_string(0));
	if(data.SizeCapacities() > 1)
		routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Demand1),
			kNullCapacitySlack, capacity_horizon_1, true, kCapacity+std::to_string(1));

  // Setting Vehicles
  for(int i = 0; i < data.SizeVehicle(); ++i){
	  routing.SetFixedCostOfVehicle(data.FixedCostVehicle(i),i);
  }

  const RoutingDimension& time_dimension = routing.GetDimensionOrDie(kTime);

  for(int vehc = 0; vehc < data.SizeVehicle(); vehc++) {
		time_dimension.CumulVar(vehc)->SetMin(data.ReadyTimeVehicle(vehc));
		routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(vehc));
		//routing.GetMutableDimension(kTime)->SetSpanCostCoefficientForVehicle(data.VehicleDistanceCost(vehc), vehc);

		routing.GetMutableDimension(kTime)->SetEndCumulVarSoftUpperBound(vehc, data.FinishTimeVehicle(vehc),FLAGS_soft_upper_bound);

		for(int i = 0; i< data.SizeCapacities(); ++i){
			const RoutingDimension& capacity_dimension = routing.GetDimensionOrDie(kCapacity+std::to_string(i));
			routing.AddVariableMinimizedByFinalizer(capacity_dimension.CumulVar(vehc));
			capacity_dimension.CumulVar(vehc)->SetMax(data.CapacitiesVehicle(vehc,0));
		}
  }

  //routing.SetArcCostEvaluatorOfAllVehicles(NewPermanentCallback(&data, &TSPTWDataDT::Distance));

  // Restriction des clients qu'un véhicule peut livrer (skill)
	if(data.SizeSkills() > 0)
		for(RoutingModel::NodeIndex i(1); i < size_matrix - 1; ++i)
			for(int vehc = 0; vehc < data.SizeVehicle(); ++vehc)
				if(!data.NodePerformedbyVehicle(i, vehc%data.SizeVehicle()))
					routing.VehicleVar(routing.NodeToIndex(i))->RemoveValue(vehc);
		//  Setting time windows
  for (RoutingModel::NodeIndex i(1); i < size_matrix - 1; ++i) {
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
          cumul_var->SetMax(due);
        }
      }

      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
			if(FLAGS_skip_node)
				if(data.SizeCapacities()>0)
					routing.AddDisjunction(*vect, data.Demand0(i,i)*FLAGS_linear_cost_skip_node+FLAGS_fix_cost_skip_node);
				else
					routing.AddDisjunction(*vect, 1*FLAGS_linear_cost_skip_node+FLAGS_fix_cost_skip_node);
    } else {
      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
      routing.AddDisjunction(*vect);
    }
  }

  for (int n = 0; n < size_rest; ++n) {
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(size_matrix);
    RoutingModel::NodeIndex rest(size_matrix + n);
    int p = 0;
    for (RoutingModel::NodeIndex i((1 + n) * size_matrix); i < (2 + n) * size_matrix; ++i, ++p) {
      (*vect)[p] = i;

      int64 index = routing.NodeToIndex(i);
      IntVar* const cumul_var = routing.CumulVar(index, kTime);
      int64 const ready = data.ReadyTime(rest);
      int64 const due = data.DueTime(rest);

      if (ready > 0) {
        cumul_var->SetMin(ready);
      }
      if (due > 0 && due < 2147483647) {
        if (FLAGS_soft_upper_bound > 0) {
          routing.SetCumulVarSoftUpperBound(i, kTime, due, FLAGS_soft_upper_bound);
        } else {
          cumul_var->SetMax(due);
        }
      }
    }
    routing.AddDisjunction(*vect);
  }

  //  Search strategy
  // routing.set_first_solution_strategy(RoutingModel::ROUTING_DEFAULT_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_GLOBAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_LOCAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_PATH_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_EVALUATOR_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_ALL_UNPERFORMED);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_BEST_INSERTION);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_GLOBAL_CHEAPEST_INSERTION);

  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GREEDY_DESCENT);
  routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GUIDED_LOCAL_SEARCH);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_SIMULATED_ANNEALING);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_TABU_SEARCH);

  // routing.SetCommandLineOption("routing_no_lns", "true");

  if (FLAGS_time_limit_in_ms > 0) {
    routing.UpdateTimeLimit(FLAGS_time_limit_in_ms);
  }

  Solver *solver = routing.solver();

  // Definition des rechargements au depot
  for(int vehc = 14; vehc < data.SizeVehicle()*(1+FLAGS_back_to_depot); vehc++) {
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(vehc - data.SizeVehicle()));
		solver->MakeLessOrEqual(time_dimension.CumulVar(vehc), time_dimension.CumulVar(vehc - data.SizeVehicle()));
		//routing.GetMutableDimension(kTime)->SetSpanCostCoefficientForVehicle(data.VehicleDistanceCost(vehc%data.SizeVehicle()), vehc);

    routing.GetMutableDimension(kTime)->SetEndCumulVarSoftUpperBound(vehc, data.FinishTimeVehicle(vehc%data.SizeVehicle()),FLAGS_soft_upper_bound);

		for(int i = 0; i< data.SizeCapacities(); ++i){
			const RoutingDimension& capacity_dimension = routing.GetDimensionOrDie(kCapacity+std::to_string(i));
			routing.AddVariableMinimizedByFinalizer(capacity_dimension.CumulVar(vehc - data.SizeVehicle()));
			solver->MakeLessOrEqual(capacity_dimension.CumulVar(vehc), capacity_dimension.CumulVar(vehc - data.SizeVehicle()));
			capacity_dimension.CumulVar(vehc)->SetMax(data.CapacitiesVehicle(vehc%data.SizeVehicle(),0));
		}
  }

  const Assignment* solution = routing.Solve(NULL);

  if (solution != NULL) {
    std::cout << "Cost: " << solution->ObjectiveValue() << std::endl;
    TSPTWSolution sol(data, &routing, solution);
    for(int route_nbr = 0; route_nbr < data.SizeVehicle()*(1+FLAGS_back_to_depot); route_nbr++) {
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        nodeIndex = data.RestShiftValue(nodeIndex.value());
        std::cout << nodeIndex << " ";
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

	if(FLAGS_vehicle_file == ""){
		operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
		operations_research::TSPTWSolver(tsptw_data);
  }else{
		operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file, FLAGS_vehicle_file);
		operations_research::TSPTWSolver(tsptw_data);
	}

  return 0;
}
