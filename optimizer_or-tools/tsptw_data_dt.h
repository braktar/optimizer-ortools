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
		LoadVehicles();
    LoadInstance(filename);
    SetRoutingDataInstanciated();
  }
  explicit TSPTWDataDT(std::string filename, std::string vehicle_file) : RoutingDataDT(0), instantiated_(false), v_instantiated_(false) {

		LoadVehicles(vehicle_file);
    LoadInstance(filename);
    SetRoutingDataInstanciated();
  }
  void LoadInstance(const std::string & filename);
  void LoadVehicles(const std::string & vehicle_file);
  void LoadVehicles();
  
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

  int64 Horizon() const {
    return horizon_;
  }

	int64 Capacity_Horizon(int64 i) const {
		if (capacity_horizon_-> size()> i)
			return capacity_horizon_->at(i);
		else
			return 0;
  }

  int64 RestShiftValue(int64 value) const {
    if (value < SizeMatrix()) {
      return value;
    }
    else {
      return (value - SizeMatrix()) / SizeMatrix() + SizeMatrix();
    }
  }

  int64 ReadyTimeVehicle(int64 i) const {
	return tsptw_vehicles_[i].ready_time;
  }

  int64 FinishTimeVehicle(int64 i) const {
	return tsptw_vehicles_[i].finish_time;
  }

  bool SkillVehicle(int64 i, int64 j) const {
	return tsptw_vehicles_[i].skills->at(j);
  }

  int64 CapacitiesVehicle(int64 i, int64 j) const {
	return tsptw_vehicles_[i].capacities->at(j);
  }

  int64 FixedCostVehicle(int64 i) const {
	return tsptw_vehicles_[i].fixed_cost;
  }

  int64 DistanceCostVehicle(int64 i) const {
	return tsptw_vehicles_[i].distance_cost;
  }

  bool NodeNeedThisSkill(RoutingModel::NodeIndex i, int64 s) const {
	return tsptw_clients_[RestShiftValue(i.value())].need_skill->at(s);
  }


  int64 ReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[RestShiftValue(i.value())].ready_time;
  }

  int64 DueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[RestShiftValue(i.value())].due_time;
  }

  int64 ServiceTime(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[RestShiftValue(i.value())].service_time;
  }

  int64 Demand0(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const { // Duplicate to add Capacity Dimension
    return tsptw_clients_[RestShiftValue(from.value())].demand->at(0);
  }

  int64 Demand1(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
    return tsptw_clients_[RestShiftValue(from.value())].demand->at(1);
  }

  /*int64 DemandThisCapacity(RoutingModel::NodeIndex to, int capacity) const {
	return tsptw_clients_[RestShiftValue(to.value())].demand->at(capacity);
  }*/

  bool NodePerformedbyVehicle(RoutingModel::NodeIndex to, int vehicle) const {
  bool temp = true;
  for(int64 i = 0; i < SizeSkills(); ++i)
		if(NodeNeedThisSkill(to, i) && !SkillVehicle(vehicle, i))
			temp = false;
  return temp;
  }

  int64 VehicleDistanceCost(int64 vehicle) const {
		return tsptw_vehicles_[vehicle].distance_cost;
  }

  // Override
  int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
    if (i.value() < SizeMatrix() && j.value() < SizeMatrix()) {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return times_.Cost(i, j);
    }
    else {
      RoutingModel::NodeIndex ii(RestShiftValue(i.value()));
      RoutingModel::NodeIndex jj(RestShiftValue(i.value()));
      return times_.Cost(ii, jj);
    }
  }

  // Override
  int64 Distance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
    if (i.value() < SizeMatrix() && j.value() < SizeMatrix()) {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(i, j);
    }
    else {
      RoutingModel::NodeIndex ii(RestShiftValue(i.value()));
      RoutingModel::NodeIndex jj(RestShiftValue(i.value()));
      return distances_.Cost(ii, jj);
    }
  }

  // Override
  int64& InternalDistance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) {
    if (i.value() < SizeMatrix() && j.value() < SizeMatrix()) {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(i,j);
    }
    else {
      RoutingModel::NodeIndex ii(RestShiftValue(i.value()));
      RoutingModel::NodeIndex jj(RestShiftValue(i.value()));
      return distances_.Cost(ii, jj);
    }
  }

  //  Transit quantity at a node "from"
  //  This is the quantity added after visiting node "from"
  int64 DistancePlusServiceTime(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Distance(from, to) + ServiceTime(from);
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

  int32 SizeVehicle() const {
    return size_vehicle_;
  }

  int32 SizeSkills() const {
    return nSkills_;
  }

  int32 SizeCapacities() const {
    return nCapacities_;
  }
  
private:
  int32 size_matrix_;
  int32 size_rest_;
  int32 size_vehicle_;
  void ProcessNewLine(char* const line);
  void ProcessNewLineVehicle(char* const line);
  void InitLoadInstance() {
    line_number_ = 0;
    visualizable_ = false;
    two_dimension_ = false;
    symmetric_ = false;
    name_ = "";
    comment_ = "";
  }
  void InitLoadVehicles() {
    v_line_number_ = 0;
  }

  //  Helper function
  int64& SetMatrix(int i, int j) {
    return distances_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64& SetTimeMatrix(int i, int j) {
    return times_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }


  bool instantiated_;
  bool v_instantiated_;
  RoutingModel::NodeIndex start_, stop_;
  struct TSPTWClient {
  TSPTWClient(int cust_no, std::vector<int64>* d, double r_t, double d_t, double s_t, std::vector<bool> * s) :
    customer_number(cust_no), demand(d), ready_time(r_t), due_time(d_t), service_time(s_t), need_skill(s) {
    }
  TSPTWClient(int cust_no, std::vector<int64>* d, double r_t, double d_t, double s_t) :
    customer_number(cust_no), demand(d), ready_time(r_t), due_time(d_t), service_time(s_t), need_skill(NULL) {}
  TSPTWClient(int cust_no, double r_t, double d_t):
    customer_number(cust_no), demand(NULL), ready_time(r_t), due_time(d_t), service_time(0.0), need_skill(NULL){
    }
  TSPTWClient(int cust_no, double r_t, double d_t, double s_t):
    customer_number(cust_no), demand(NULL), ready_time(r_t), due_time(d_t), service_time(s_t), need_skill(NULL){
    }
    int customer_number;
    std::vector<int64>* demand;
    int64 ready_time;
    int64 due_time;
    int64 service_time;
    std::vector<bool> *need_skill;
  };
  struct TSPTWVehicle {
		TSPTWVehicle() :
		vehicle_number(0), fixed_cost(0), distance_cost(0), ready_time(0), finish_time(2147483647), skills(NULL), capacities(NULL){}
		TSPTWVehicle(int v_no, int f_co, int d_co, int r_t, int f_t, std::vector<bool> *s, std::vector<int64> *c):
		vehicle_number(v_no), fixed_cost(f_co), distance_cost(d_co), ready_time(r_t), finish_time(f_t), skills(s), capacities(c){}

		int vehicle_number;
		int fixed_cost;
		int distance_cost;
		int ready_time;
		int finish_time;
		std::vector<bool> *skills;
    std::vector<int64> *capacities;
  };

  std::vector<TSPTWClient> tsptw_clients_;
  std::vector<TSPTWVehicle> tsptw_vehicles_;
  std::string details_;
  std::string filename_;
  std::string vehicle_file_;
  int64 horizon_;
  std::vector<int64>* capacity_horizon_;
  bool visualizable_;
  bool two_dimension_;
  bool symmetric_;
  int64 nSkills_;
  int64 nCapacities_;
    
  int line_number_;
  int v_line_number_;
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
  FileLineReader reader(filename.c_str());
  reader.set_line_callback(NewPermanentCallback(
                           this,
                           &TSPTWDataDT::ProcessNewLine));
  reader.Reload();
  if (!reader.loaded_successfully()) {
    LOG(ERROR) << "Could not open TSPTW file " << filename;
  }

  // Problem size
  size_ = size_matrix_ + size_rest_ * size_matrix_;

  // Compute horizon
  for (int32 i = 0; i < size_matrix_ + size_rest_; ++i) {
    horizon_ = std::max(horizon_, tsptw_clients_[i].due_time);
  }

  // Setting start: always first matrix node
  start_ = RoutingModel::NodeIndex(tsptw_clients_[0].customer_number);

  // Setting stop: always last matrix node
  stop_ = RoutingModel::NodeIndex(tsptw_clients_[SizeMatrix() - 1].customer_number);

  filename_ = filename;
  instantiated_ = true;
}

void TSPTWDataDT::LoadVehicles() {
  InitLoadVehicles();
  size_vehicle_ = 1;
  nCapacities_ = 0;
  nSkills_ = 0;
  tsptw_vehicles_.push_back(TSPTWVehicle());
  vehicle_file_ = "";
  v_instantiated_ = true;
}

void TSPTWDataDT::LoadVehicles(const std::string & vehicle_file) {
  InitLoadVehicles();
  size_vehicle_ = 0;
  nCapacities_ = 0;
  nSkills_ = 0;
  FileLineReader reader(vehicle_file.c_str());
  reader.set_line_callback(NewPermanentCallback(
													 this,
													 &TSPTWDataDT::ProcessNewLineVehicle));

  reader.Reload();

  for (int32 i = 0 ; i < size_vehicle_; ++i)
		for (int32 j = 0; j < nCapacities_; ++j)
			capacity_horizon_[j] = std::max(capacity_horizon_[j], tsptw_vehicles_[i].capacities[j]);

  if (!reader.loaded_successfully()) {
		LOG(ERROR) << "Could not open vehicle file " << vehicle_file;
  }

  vehicle_file_ = vehicle_file;
  v_instantiated_ = true;
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
    CreateRoutingData(size_matrix_ + size_rest_);
    // Matrix default values
    for (int64 i=0; i < size_matrix_ + size_rest_; ++i) {
      for (int64 j=0; j < size_matrix_ + size_rest_; ++j) {
        SetMatrix(i, j) = 0;
        SetTimeMatrix(i, j) = 0;
      }
    }
  }
  else if (line_number_ > 2 && line_number_ <= 2 + SizeMatrix()) {
    CHECK_EQ(words.size(), SizeMatrix() * 2) << "Distance matrix in TSPTW instance file is ill formed : " << line_number_;
    for (int j = 0; j < SizeMatrix(); ++j) {
      SetMatrix(line_number_ - 3, j) = static_cast<int64>(atof(words[j*2].c_str()));
      SetTimeMatrix(line_number_ - 3, j) = static_cast<int64>(atof(words[j*2+1].c_str()));
    }
  }
  else if (words.size() == 3 && line_number_ > 2 + SizeMatrix() && line_number_ <= 2 + SizeMatrix() + Size()) {
    tsptw_clients_.push_back(TSPTWClient(line_number_ - 3 - SizeMatrix(),
                                         atof(words[0].c_str()),
                                         atof(words[1].c_str()),
                                         atof(words[2].c_str())
    ));
  }
  else if (words.size() == 5 && line_number_ > 2 + SizeMatrix() && line_number_ <= 2 + SizeMatrix() + Size()) {
    std::vector<bool>* t_s(new std::vector<bool>);
    std::vector<int64>* t_c(new std::vector<int64>);
		for(int i = 0; i < SizeSkills() ; ++i)
			if(atof(words[3+SizeCapacities()+i].c_str()) == 1)
				t_s->push_back(true);
			else
				t_s->push_back(false);
		for(int i = 0; i < SizeCapacities() ; ++i)
			t_c->push_back(atof(words[3+i].c_str())*1000);

		tsptw_clients_.push_back(TSPTWClient(line_number_ - 3 - SizeMatrix(),
																					 t_c,
		                                       atof(words[0].c_str()),
		                                       atof(words[1].c_str()),
		                                       atof(words[2].c_str()),
																					 t_s
		));
  }
  else if (line_number_ > 2 + SizeMatrix() && line_number_ <= 2 + SizeMatrix() + Size()) {
		CHECK_EQ(words.size(), 5) << "Time window in TSPTW instance file is ill formed : " << line_number_;
  }
  else {
    VLOG(0) << "Unexpected line :" << line_number_;
  }
}  //  void ProcessNewLine(char* const line)

void TSPTWDataDT::ProcessNewLineVehicle(char* const line) {
  ++v_line_number_;

  static const char kWordDelimiters[] = " ";
  std::vector<std::string> words = strings::Split(line, kWordDelimiters, strings::SkipEmpty());

  static const int DSU_data_tokens = 7;
  static const int DSU_last_customer = 999;

  //  Empty lines
  if (words.size() == 0) {
    return;
  }
  else if (v_line_number_ == 1) {
		size_vehicle_ = atoi32(words[0]);
  }
  else if (v_line_number_ == 2) {
    nSkills_ = atoi32(words[0]);
  }
  else if (v_line_number_ == 3) {
    nCapacities_ = atoi32(words[0]);
    CHECK_LE(nCapacities_, 2) << "Too many capacities : " << nCapacities_<<"/2" ;
    for(int i = 0; i< nCapacities_; ++i)
			capacity_horizon_->push_back(0);
  }
  else if (v_line_number_ > 3 && v_line_number_ <= 3 + SizeVehicle()) {
		CHECK_EQ(words.size(), 4 + SizeCapacities() + SizeSkills() ) << "Vehicule Configuration don't match : " << v_line_number_;
		std::vector<bool>* t_s(new std::vector<bool>);
    std::vector<int64>* t_c(new std::vector<int64>);
	  for(int i = 0; i < SizeSkills() ; ++i)
	    if(atof(words[4+SizeCapacities()+i].c_str()) == 1)
			  t_s->push_back(true);
	    else
			  t_s->push_back(false);
	  for(int i = 0; i < SizeCapacities() ; ++i)
	    t_c->push_back(atof(words[4+i].c_str()));

    tsptw_vehicles_.push_back(TSPTWVehicle(v_line_number_ - 4,
                                           atof(words[0].c_str()),
																					 atof(words[1].c_str()),
		                                       atof(words[2].c_str()),
																					 atof(words[3].c_str()),
																					 t_s,
																					 t_c
    ));
  }
  else {
    VLOG(0) << "Unexpected line :" << v_line_number_;
  }
}  //  void ProcessNewLineVehicle(char* const line)

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H