#include <global_planner/cameraGainEvaluator.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace globalPlanner {
namespace {struct BudgetExceeded:std::runtime_error {using std::runtime_error::runtime_error;};}
void CameraSharedBudget::check() const {
 if(cancelled)throw BudgetExceeded("cancelled");
 if(std::chrono::steady_clock::now()>=deadline)throw BudgetExceeded("absolute_deadline_exhausted");
 if(externalCheck)externalCheck();
}
CameraGainEvaluator::CameraGainEvaluator(const mapManager::OccupancyMapSnapshot& m,const CameraGainConfig& c):map_(m),config_(c) {
 uint64_t count=1;
 for(int a=0;a<3;++a) {
  if(m.dimensions[a]<=0 || count>std::numeric_limits<uint32_t>::max()/uint64_t(m.dimensions[a]))
   throw std::invalid_argument("invalid camera dimensions or address overflow");
  count*=m.dimensions[a];
 }
 if(!m.mapMin.allFinite() || !m.mapMax.allFinite() || !std::isfinite(m.resolution) || m.resolution<=0 ||
    (m.dimensions.array()<=0).any() || count<=0 || uint64_t(count)>std::numeric_limits<uint32_t>::max() ||
    m.occupancy.size()!=uint64_t(count) || m.inflated.size()!=uint64_t(count) ||
    !std::isfinite(m.pMinLog) || !std::isfinite(m.pOccLog) || m.pOccLog<m.pMinLog)
  throw std::invalid_argument("invalid frozen camera map");
 if(!std::isfinite(c.fx) || !std::isfinite(c.fy) || !std::isfinite(c.cx) || !std::isfinite(c.cy) ||
    !std::isfinite(c.range) || c.fx<=0 || c.fy<=0 || c.range<=0 || c.cols<=0 || c.rows<=0 ||
    c.margin<0 || c.pixelSkip<=0 || !c.bodyToCamera.allFinite() ||
    !c.planningMin.allFinite() || !c.planningMax.allFinite() || (c.planningMin.array()>c.planningMax.array()).any())
  throw std::invalid_argument("invalid frozen camera configuration");
 if((c.bodyToCamera.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-12 ||
    (c.bodyToCamera.topLeftCorner<3,3>().transpose()*c.bodyToCamera.topLeftCorner<3,3>()-Eigen::Matrix3d::Identity()).norm()>1e-9 ||
    std::abs(c.bodyToCamera.topLeftCorner<3,3>().determinant()-1)>1e-9)
  throw std::invalid_argument("camera extrinsic must be a rigid transform");
 std::vector<uint8_t> region[3];
 for(int a=0;a<3;++a) {
  region[a].resize(m.dimensions[a]);
  for(int i=0;i<m.dimensions[a];++i) {
   const double centre=m.mapMin[a]+(double(i)+.5)*m.resolution;
   region[a][i]=centre>=c.planningMin[a] && centre<=c.planningMax[a];
  }
 }
 opticalState_.resize(count);
 for(int x=0;x<m.dimensions.x();++x)for(int y=0;y<m.dimensions.y();++y)for(int z=0;z<m.dimensions.z();++z) {
  const auto id=address(Eigen::Vector3i(x,y,z));const auto value=m.occupancy[id];
  opticalState_[id]=value>=m.pOccLog?1:(value<m.pMinLog && region[0][x] && region[1][y] && region[2][z]?-1:0);
 }
 for(int v=c.margin;v<c.rows-c.margin;v+=c.pixelSkip)
  for(int u=c.margin;u<c.cols-c.margin;u+=c.pixelSkip) {
   Eigen::Vector3d ray((u-c.cx)/c.fx,(v-c.cy)/c.fy,1);
   const double norm=std::sqrt(ray.x()*ray.x()+ray.y()*ray.y()+1.0);
   rays_.push_back(ray/norm);
  }
}
void CameraGainEvaluator::Work::check() const {
 if(shared)shared->check();
 if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()>=budget.maxMilliseconds)
  throw BudgetExceeded("wall_budget_exhausted");
}
void CameraGainEvaluator::Work::visit() {
 if(shared){if(shared->voxelVisits>=shared->maxVoxelVisits)throw BudgetExceeded("shared_voxel_budget_exhausted");++shared->voxelVisits;}
 if(visits>=budget.maxVoxelVisits)throw BudgetExceeded("voxel_budget_exhausted");
 ++visits;if((visits&255)==0)check();
}
bool CameraGainEvaluator::inBounds(const Eigen::Vector3i& i) const {
 return (i.array()>=0).all() && (i.array()<map_.dimensions.array()).all();
}
uint32_t CameraGainEvaluator::address(const Eigen::Vector3i& i) const {
 return uint32_t(uint64_t(i.x())*map_.dimensions.y()*map_.dimensions.z()+uint64_t(i.y())*map_.dimensions.z()+i.z());
}
CameraGainEvaluator::AddressSet CameraGainEvaluator::visible(const Eigen::Vector3d& p,double yaw,Work& work,CameraVisibilityTrace* trace,bool dependencies) const {
 if(!p.allFinite() || !std::isfinite(yaw))throw std::invalid_argument("nonfinite camera pose");
 work.check();
 const double cs=std::cos(yaw),sn=std::sin(yaw);
 Eigen::Matrix3d body;body<<cs,-sn,0,sn,cs,0,0,0,1;
 const Eigen::Vector3d origin=p+body*config_.bodyToCamera.topRightCorner<3,1>();
 const Eigen::Matrix3d rotation=body*config_.bodyToCamera.topLeftCorner<3,3>();
 const Eigen::Vector3d start=(origin-map_.mapMin)/map_.resolution;
 // Reject out-of-map origin before integer conversion.
 if((start.array()<0).any() || (start.array()>=map_.dimensions.cast<double>().array()).any())return {};
 const Eigen::Vector3i sensor=start.array().floor().cast<int>();
 if(trace){trace->regionMin=sensor;trace->regionMax=sensor;}
 AddressSet result;std::unordered_set<uint32_t> endpoints;
 // Local generation marks are ray-endpoint scratch, not cached map/visibility results.
 const bool denseMarks=map_.occupancy.size()<=work.endpointMarkBudgetBytes/sizeof(uint32_t);
 if(denseMarks) {
  if(work.endpointMarks.empty())work.endpointMarks.assign(map_.occupancy.size(),0);
  if(++work.endpointGeneration==0){std::fill(work.endpointMarks.begin(),work.endpointMarks.end(),0);work.endpointGeneration=1;}
 }
 struct Endpoint {Eigen::Vector3d point;Eigen::Vector3i index;};
 std::vector<Endpoint> representatives;representatives.reserve(2048);
 const auto projected=work.budget.profile?std::chrono::steady_clock::now():work.started;
 for(size_t n=0;n<rays_.size();++n) {
  if((n&255)==0)work.check();
  const Eigen::Vector3d end=(origin+config_.range*(rotation*rays_[n])-map_.mapMin)/map_.resolution;
  if(!end.allFinite() || (end.array()<0).any() || (end.array()>=map_.dimensions.cast<double>().array()).any())continue;
  const Eigen::Vector3i terminal=end.array().floor().cast<int>();
  const auto endpointId=address(terminal);
  if(denseMarks) {
   if(work.endpointMarks[endpointId]==work.endpointGeneration)continue;
   work.endpointMarks[endpointId]=work.endpointGeneration;
  }else if(!endpoints.insert(endpointId).second)continue;
  if(terminal==sensor)continue;
  if(trace){trace->regionMin=trace->regionMin.cwiseMin(terminal);trace->regionMax=trace->regionMax.cwiseMax(terminal);}
  representatives.push_back({end,terminal});
 }
 const auto traversal=work.budget.profile?std::chrono::steady_clock::now():work.started;
 if(work.budget.profile)work.endpointMs+=std::chrono::duration<double,std::milli>(traversal-projected).count();
 work.representativeCount+=representatives.size();
 for(const auto& ray:representatives) {
  const auto& end=ray.point;const auto& terminal=ray.index;
  Eigen::Vector3i current=sensor,step=Eigen::Vector3i::Zero();
  const Eigen::Vector3d direction=end-start;
  Eigen::Vector3d delta=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),maximum=delta;
  for(int a=0;a<3;++a) {
   step[a]=(terminal[a]>sensor[a])-(terminal[a]<sensor[a]);
   if(step[a] && direction[a]!=0) {
    maximum[a]=(sensor[a]+(step[a]>0?1:0)-start[a])/direction[a];
    delta[a]=std::abs(1.0/direction[a]);
   }
  }
  const int safety=(terminal-sensor).cwiseAbs().sum()+3;
  int64_t linear=address(sensor);
  const int64_t stride[3]={int64_t(map_.dimensions.y())*map_.dimensions.z(),map_.dimensions.z(),1};
  for(int k=0;k<safety;++k) {
   int a=maximum.x()<maximum.y() && maximum.x()<maximum.z()?0:(maximum.y()<maximum.z()?1:2);
   current[a]+=step[a];linear+=step[a]*stride[a];maximum[a]+=delta[a];work.visit();
   if(!inBounds(current))break;
   const auto id=uint32_t(linear);const auto state=opticalState_[id];
   if(state==1){if(trace&&dependencies)trace->blockers.push_back(id);break;}
   if(trace&&dependencies)trace->transparent.push_back(id);
   if(state==-1)result.insert(id);
   if(current==terminal)break;
   if(k==safety-1)throw std::runtime_error("camera traversal safety bound exceeded");
  }
 }
 if(work.budget.profile)work.traversalMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-traversal).count();
 return result;
}
std::vector<CameraVisibilityTrace> CameraGainEvaluator::traceSamples(const std::vector<PathGainWaypoint>& samples,bool dependencies) const {
 Work work{{},std::chrono::steady_clock::now(),0};std::vector<CameraVisibilityTrace> out;out.reserve(samples.size());
 for(const auto& p:samples){
  out.emplace_back();auto& t=out.back();t.visible=visible(p.position,p.yaw,work,&t,dependencies);
  for(auto* v:{&t.transparent,&t.blockers}){std::sort(v->begin(),v->end());v->erase(std::unique(v->begin(),v->end()),v->end());v->shrink_to_fit();}
 }
 return out;
}
CameraGainEvaluator::AddressSet CameraGainEvaluator::visibleUnknown(const Eigen::Vector3d& p,double yaw) const {
 Work work{{},std::chrono::steady_clock::now(),0};return visible(p,yaw,work);
}
CameraPoseSetsResult CameraGainEvaluator::evaluatePoseSets(const std::vector<PathGainWaypoint>& poses,uint64_t version,CameraSharedBudget& budget) const {
 return evaluatePoseSets(poses,version,budget,32u*1024u*1024u);
}
CameraPoseSetsResult CameraGainEvaluator::evaluatePoseSets(const std::vector<PathGainWaypoint>& poses,uint64_t version,CameraSharedBudget& budget,size_t endpointMarkBudgetBytes) const {
 CameraPoseSetsResult out;const auto begin=std::chrono::steady_clock::now();
 try {
  budget.check();
  if(version!=map_.version)throw std::invalid_argument("map_version_mismatch");
  Work work{{},begin,0};work.shared=&budget;work.endpointMarkBudgetBytes=endpointMarkBudgetBytes;
  for(const auto& pose:poses){
   budget.check();if(budget.samples>=budget.maxSamples)throw BudgetExceeded("shared_sample_budget_exhausted");
   ++budget.samples;out.poseSets.push_back(visible(pose.position,pose.yaw,work));budget.check();
  }
  budget.check();out.mapVersion=version;out.valid=true;
 }catch(const std::bad_alloc&){out.poseSets={};out.mapVersion=0;out.failureReason="memory_limit";}
 catch(const std::exception& e){out.poseSets={};out.mapVersion=0;out.failureReason=e.what();}
 out.evaluationMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return out;
}
CameraGainResult CameraGainEvaluator::evaluateSamples(const std::vector<PathGainWaypoint>& samples,double time,uint64_t version,const CameraGainBudget& budget) const {
 CameraGainResult result;Work work{budget,std::chrono::steady_clock::now(),0};
 try {
  if(version!=map_.version)throw std::invalid_argument("map_version_mismatch");
  if(samples.empty())throw std::invalid_argument("empty_camera_samples");
  if(samples.size()>budget.maxSamples)throw BudgetExceeded("sample_budget_exhausted");
  if(std::isnan(budget.maxMilliseconds) || budget.maxMilliseconds<0 || !std::isfinite(time) || time<0)
   throw std::invalid_argument("invalid_time_or_budget");
  AddressSet history;result.gain.mapVersion=version;result.gain.estimatedTime=time;
  for(const auto& sample:samples) {
   const auto seen=visible(sample.position,sample.yaw,work);uint64_t marginal=0;
   result.gain.rawGain+=seen.size();for(auto id:seen)marginal+=history.insert(id).second;
   result.gain.marginalGains.push_back(marginal);
  }
  work.check();result.gain.sampleCount=samples.size();result.gain.uniqueGain=history.size();
  result.gain.uniqueAddresses.assign(history.begin(),history.end());std::sort(result.gain.uniqueAddresses.begin(),result.gain.uniqueAddresses.end());
  result.gain.duplicateRatio=result.gain.rawGain?1.0-double(result.gain.uniqueGain)/result.gain.rawGain:0;
  result.gain.uniqueUtility=time>1e-6?result.gain.uniqueGain/time:0;result.gain.valid=true;
 }catch(const std::exception& e){result.gain=PathGainEvaluation();result.failureReason=e.what();}
 result.voxelVisits=work.visits;result.endpointMs=work.endpointMs;result.traversalMs=work.traversalMs;result.representativeCount=work.representativeCount;result.evaluationMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-work.started).count();
 return result;
}
}
