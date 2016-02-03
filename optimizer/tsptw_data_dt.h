#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H

#include <ostream>
#include <iomanip>
#include <vector>

#include "constraint_solver/routing.h"
#include "base/filelinereader.h"
#include "base/split.h"
#include "base/strtoint.h"


#include "routing_data_dt.h"

namespace operations_research {

class TSPTWDataDT : public RoutingDataDT {
public:
  explicit TSPTWDataDT(std::string filename) : RoutingDataDT(0), instantiated_(false) {
    LoadInstance(filename);
    SetRoutingDataInstanciated();
  }
  void LoadInstance(const std::string & filename);
  
  void SetStart(RoutingModel::NodeIndex s) {
    CHECK_LT(s, Size());
    start_ = s;
  }

  void SetStop(RoutingModel::NodeIndex s) {
    CHECK_LT(s, Size());
    stop_ = s;
  }

  RoutingModel::NodeIndex Start() const {
    return start_;
  }

  RoutingModel::NodeIndex Stop() const {
    return stop_;
  }

  RoutingModel::NodeIndex VehicleStart(int i) const {
    return RoutingModel::NodeIndex(tsptw_vehicles_[i].start_index);
  }

  RoutingModel::NodeIndex VehicleEnd(int i) const {
    if(tsptw_vehicles_[i].end_index == -1)
      return RoutingModel::NodeIndex(size_matrix_ - 1);
    return RoutingModel::NodeIndex(tsptw_vehicles_[i].end_index);
  }

  int64 Horizon() const {
    return horizon_;
  }

  int64 ReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].ready_time;
  }

  int64 DueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].due_time;
  }

  int64 ServiceTime(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].service_time;
  }

  int64 NumberTimeWindows(int i) const {
    return tsptw_tw_clients_[i];
  }

  // Override
  int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return times_.Cost(RoutingModel::NodeIndex(tsptw_clients_[i.value()].customer_number), RoutingModel::NodeIndex(tsptw_clients_[j.value()].customer_number));
  }

  // Override
  int64 Distance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(RoutingModel::NodeIndex(tsptw_clients_[i.value()].customer_number), RoutingModel::NodeIndex(tsptw_clients_[j.value()].customer_number));
  }

  // Override
  int64& InternalDistance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(RoutingModel::NodeIndex(tsptw_clients_[i.value()].customer_number), RoutingModel::NodeIndex(tsptw_clients_[j.value()].customer_number));
  }

  //  Transit quantity at a node "from"
  //  This is the quantity added after visiting node "from"
  int64 DistancePlusServiceTime(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Distance(from, to) + ServiceTime(from);
  }

  bool CustomerAlreadyExist(int custom){
    for (int i = 0; i < parseIndex_.size() ; ++i)
      if(parseIndex_[i] == custom){
        return true;
      }
    return false;
  }

  //  Transit quantity at a node "from"
  //  This is the quantity added after visiting node "from"
  int64 TimePlusServiceTime(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Time(from, to) + ServiceTime(from);
  }

  int64 TimePlus(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Time(from, to);
  }

  int64 ReadyTimeVehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].ready_time;
  }

  int64 FinishTimeVehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].finish_time;
  }

  int64 Demand0(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const { // Duplicate to add Capacity Dimension
    return tsptw_clients_[from.value()].demand->at(0);
  }

  int64 Demand1(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
    return tsptw_clients_[from.value()].demand->at(1);
  }

  int64 Demand2(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
    return tsptw_clients_[from.value()].demand->at(2);
  }

  int64 CapacitiesVehicle(int64 i, int64 j) const {
    return tsptw_vehicles_[i%size_vehicle_].capacities->at(j);
  }

  int64 Capacity0Vehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].capacities->at(0);
  }

  int64 Capacity1Vehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].capacities->at(1);
  }

  int64 Capacity2Vehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].capacities->at(2);
  }

  int64 VehicleDistanceCost(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].distance_cost;
  }

  int64 VehicleTimeCost(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].time_cost;
  }

  int64 MaxVehicleCapacity(int64 capa) const {
    int cap_max_ = 0;
    for(int vehc = 0; vehc < size_vehicle_; ++vehc)
      cap_max_ = std::max((int)CapacitiesVehicle(vehc, capa), cap_max_);
    return cap_max_;
  }

  int64 FixedCostVehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].fixed_cost;
  }

  int64 DistanceCostVehicle(int64 i) const {
    return tsptw_vehicles_[i%size_vehicle_].distance_cost;
  }

  void PrintLIBInstance(std::ostream& out) const;
  void PrintDSUInstance(std::ostream& out) const;
  void WriteLIBInstance(const std::string & filename) const;
  void WriteDSUInstance(const std::string & filename) const;

  int32 SizeMatrix() const {
    return size_matrix_;
  }

  int32 SizeRest() const {
    return size_rest_;
  }

  int32 SizeTimeWindows() const {
    return size_tws_;
  }

  int32 SizeVehicle() const {
    return size_vehicle_;
  }

  int32 SizeCapacities() const {
    return size_capacities_;
  }

  std::vector<RoutingModel::NodeIndex>* VectorNode(int custom) const {
      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>();
      int i = 0;
      while(vect->size() < tsptw_tw_clients_[custom]){
        if(tsptw_clients_[i].customer_number == custom){
          vect->push_back(RoutingModel::NodeIndex(i));
        }
        ++i;
      }
      return vect;
  }
  
private:
  int32 size_matrix_;
  int32 size_rest_;
  int32 size_tws_;
  void ProcessNewLine(char* const line);
  void InitLoadInstance() {
    line_number_ = 0;
    visualizable_ = false;
    two_dimension_ = false;
    symmetric_ = false;
    name_ = "";
    comment_ = "";
  }
  
  //  Helper function
  int64& SetMatrix(int i, int j) {
    return distances_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64& SetTimeMatrix(int i, int j) {
    return times_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }


  bool instantiated_;
  RoutingModel::NodeIndex start_, stop_;
  struct TSPTWClient {
    TSPTWClient(int cust_no, std::vector<int64> *d, double r_t, double d_t, double s_t) :
    customer_number(cust_no), demand(d), ready_time(r_t), due_time(d_t), service_time(s_t) {}
    TSPTWClient(int cust_no, double r_t, double d_t):
    customer_number(cust_no), demand(NULL), ready_time(r_t), due_time(d_t), service_time(0.0){
    }
    TSPTWClient(int cust_no, double r_t, double d_t, double s_t):
    customer_number(cust_no), demand(NULL), ready_time(r_t), due_time(d_t), service_time(s_t){
    }
    int customer_number;
    std::vector<int64> *demand;
    int64 ready_time;
    int64 due_time;
    int64 service_time;
  };
  struct TSPTWVehicle {
    TSPTWVehicle() :
    vehicle_number(0), start_index(0), end_index(-1), fixed_cost(1), distance_cost(1), time_cost(1), ready_time(0), finish_time(2147483647), capacities(NULL){}
    TSPTWVehicle(int v_no, int s_i, int e_i, int f_co, int d_co, int t_co, int r_t, int f_t, std::vector<int64> *c):
    vehicle_number(v_no), start_index(s_i), end_index(e_i), fixed_cost(f_co), distance_cost(d_co), time_cost(t_co), ready_time(r_t), finish_time(f_t), capacities(c){}
    int vehicle_number;
    int start_index;
    int end_index;
    int fixed_cost;
    int time_cost;
    int distance_cost;
    int ready_time;
    int finish_time;
    std::vector<int64> *capacities;
  };

  std::vector<TSPTWClient> tsptw_clients_;
  std::vector<int> tsptw_tw_clients_;
  std::vector<TSPTWVehicle> tsptw_vehicles_;
  std::vector<int> parseIndex_;
  std::string details_;
  std::string filename_;
  int64 horizon_;
  std::vector<int64> *capacity_horizon_;
  int64 size_vehicle_;
  int64 size_capacities_;
  bool visualizable_;
  bool two_dimension_;
  bool symmetric_;
    
  int line_number_;
  std::string comment_;
};

// Parses a file in López-Ibáñez-Blum or
// da Silva-Urrutia formats and loads the coordinates.
// Note that the format is only partially checked:
// bad inputs might cause undefined behavior.
void TSPTWDataDT::LoadInstance(const std::string & filename) {

  InitLoadInstance();
  size_ = 0;
  size_matrix_ = 0;
  size_tws_ = 0;
  size_vehicle_ = 0;
  size_capacities_ = 0;
  capacity_horizon_ = new std::vector<int64>;
  FileLineReader reader(filename.c_str());

  reader.set_line_callback(NewPermanentCallback(
                           this,
                           &TSPTWDataDT::ProcessNewLine));

  reader.Reload();
  if (!reader.loaded_successfully()) {
    LOG(ERROR) << "Could not open TSPTW file " << filename;
  }
  // Problem size
  size_ = size_tws_; // + size_rest_

  // Compute horizon
  for (int32 i = 0; i < size_matrix_ + size_rest_; ++i) {
    horizon_ = std::max(horizon_, tsptw_clients_[i].due_time);
  }

  for (int32 i = 0 ; i < size_vehicle_; ++i){
    for (int32 j = 0; j < size_capacities_; ++j){
      long long int cap_h = capacity_horizon_->at(j);
      capacity_horizon_->at(j) = std::max(capacity_horizon_->at(j), tsptw_vehicles_.at(i).capacities->at(j));
    }
  }


  // Setting start: always first matrix node
  start_ = RoutingModel::NodeIndex(tsptw_clients_[0].customer_number);

  // Setting stop: always last matrix node
  stop_ = RoutingModel::NodeIndex(SizeTimeWindows() - 1);

  filename_ = filename;
  instantiated_ = true;
}

void TSPTWDataDT::ProcessNewLine(char* const line) {
  ++line_number_;

  static const char kWordDelimiters[] = " ";
  std::vector<std::string> words = strings::Split(line, kWordDelimiters, strings::SkipEmpty());

  static const int DSU_data_tokens = 7;
  static const int DSU_last_customer = 999;
  //  Empty lines
  if (words.size() == 0) {
    return;
  }
  else if (line_number_ == 1) {
    size_matrix_ = atoi32(words[0]);
  }
  else if (line_number_ == 2) {
    size_rest_ = atoi32(words[0]);
  }
  else if (line_number_ == 3 && words.size() == 1) {
    size_tws_ = atoi32(words[0]);
    CreateRoutingData(size_tws_); // Artificial problem size
    // Matrix default values
    for (int64 i=0; i < size_tws_; ++i) {
      for (int64 j=0; j < size_tws_; ++j) {
        SetMatrix(i, j) = 0;
        SetTimeMatrix(i, j) = 0;
      }
    }
  }
  else if (line_number_ == 4) {
    size_vehicle_ = atoi32(words[0]);
  }
  else if (line_number_ == 5) {
    size_capacities_ = atoi32(words[0]);
    for(int i = 0; i< size_capacities_; ++i)
      capacity_horizon_->push_back(0);
  }
  else if (line_number_ > 5 && line_number_ <= 5 + SizeMatrix()) {
    CHECK_EQ(words.size(), SizeMatrix() * 2) << "Distance matrix in TSPTW instance file is ill formed : " << line_number_;
    for (int j = 0; j < SizeMatrix(); ++j) {
      SetMatrix(line_number_ - 6, j) = static_cast<int64>(atof(words[j*2].c_str()));
      SetTimeMatrix(line_number_ - 6, j) = static_cast<int64>(atof(words[j*2+1].c_str()));
    }
  }
  else if (line_number_ > 5 + SizeMatrix() && line_number_ <= 5 + SizeMatrix() + SizeTimeWindows() + SizeRest()) {
    CHECK_EQ(words.size(), 4 + SizeCapacities() ) << "Time window in TSPTW instance file is ill formed : " << line_number_;

    std::vector<int64>* t_c(new std::vector<int64>);
    for(int i = 0; i < SizeCapacities() ; ++i)
        t_c->push_back(atof(words[4+i].c_str())*10);

    int index = atof(words[0].c_str());
    if(index == 0 || !CustomerAlreadyExist(index)){
      parseIndex_.push_back(index);
      tsptw_tw_clients_.push_back(1);
      index = parseIndex_.size()-1;
    }
    else {
      int i = 0;
      while(parseIndex_[i] != index)
        ++i;
      index = i;
      tsptw_tw_clients_[index] = ++tsptw_tw_clients_[index];
    }
    tsptw_clients_.push_back(TSPTWClient(index,
                                         t_c,
                                         atof(words[1].c_str()),
                                         atof(words[2].c_str()),
                                         atof(words[3].c_str())
    ));
  }
  else if (line_number_ > 5 + SizeMatrix() + SizeTimeWindows() + SizeRest() && line_number_ <= 5 + SizeMatrix() + SizeTimeWindows() + SizeRest() + SizeVehicle() ) {
    CHECK_EQ(words.size(), 7 + SizeCapacities()) << "Vehicle in TSPTW instance file is ill formed : " << line_number_;
    std::vector<int64>* t_c(new std::vector<int64>);
    for(int i = 0; i < SizeCapacities() ; ++i)
      t_c->push_back(atof(words[7+i].c_str())*10);

    tsptw_vehicles_.push_back(TSPTWVehicle(line_number_ - 6 - SizeMatrix() - SizeTimeWindows() - SizeRest(),
                                           atof(words[0].c_str()),
                                           atof(words[1].c_str()),
                                           atof(words[2].c_str()),
                                           atof(words[3].c_str()),
                                           atof(words[4].c_str()),
                                           atof(words[5].c_str()),
                                           atof(words[6].c_str()),
                                           t_c
    ));
  }
  else {
    VLOG(0) << "Unexpected line :" << line_number_;
  }
}  //  void ProcessNewLine(char* const line)

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H