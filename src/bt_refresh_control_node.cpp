#include "refresh_ros2/bt_refresh_control_node.hpp"

namespace ReFRESH_BT {
void DeciderNode::halt() {
  haltChild(indActive_);
  // retry counter resets on every halt.
  nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
  indActive_ = -1;
  setStatus(NodeStatus::IDLE);
}

NodeStatus DeciderNode::turnOnBest() {
  std::vector<ReFRESH_Cost> moduleCost_;
  size_t ind;
  float pWeight_ = getInput<float>("performance_weight").value();
  float rWeight_ = getInput<float>("resource_weight").value();
  unsigned int retries = getInput<unsigned int>("retries").value();
  for (ind = 0; ind < childrenCount(); ind++) {
    // this list is for inactive modules.
    if (ind == indActive_) continue;
    // this module has exceeded max retry limits. Skip it.
    if (nRetry_[ind] > retries) continue;

    ReFRESH_Module* tryConvert = dynamic_cast<ReFRESH_Module*>(children_nodes_[ind]);
    // improper type, try to tick once and check result.
    if (tryConvert == nullptr) {
      NodeStatus childStatus = children_nodes_[ind]->executeTick();
      float pCost_, rCost_;
      switch (childStatus) {
        case NodeStatus::SUCCESS:
          // this is a fall-back node, anyway.
          return NodeStatus::SUCCESS;
          break;

        case NodeStatus::FAILURE:
          pCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
          rCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
          moduleCost_.push_back(ReFRESH_Cost(ind, pCost_, rCost_));
          moduleCost_.back().setWeights(pWeight_, rWeight_);
          break;

        case NodeStatus::RUNNING:
          // not a refresh module. if node status is not-failure then make its cost 1-eps (least
          // considered)
          pCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
          rCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
          haltChild(ind);
          moduleCost_.push_back(ReFRESH_Cost(ind, pCost_, rCost_));
          moduleCost_.back().setWeights(pWeight_, rWeight_);
          break;

        default:
          throw BT::LogicError("A child node must never return IDLE");
          break;
      }
      continue;
    }

    // proper type, use speialized function
    std::tuple<NodeStatus, float, float> esCost = tryConvert->estimate();
    // ES returned FAILURE (hard failure), skipping
    if (std::get<0>(esCost) == NodeStatus::FAILURE) continue;
    moduleCost_.push_back(ReFRESH_Cost(ind, std::get<1>(esCost), std::get<2>(esCost)));
    moduleCost_.back().setWeights(pWeight_, rWeight_);
  }
  // sort moduleCost_
  std::stable_sort(moduleCost_.begin(), moduleCost_.end());
  // check feasibility in cost-ascending order
  ind = 0;
  bestPossible_ = indActive_;
  bool bestPossibleSet = false;
  while (true) {
    if (ind >= moduleCost_.size()) {
      // no candidate satisfies the requirements
      return NodeStatus::FAILURE;
    }
    if (!moduleCost_[ind].feasible()) {
      if (!bestPossibleSet && moduleCost_[ind].resourceFeasible()) {
        bestPossibleSet = true;
        bestPossible_ = moduleCost_[ind].whichChild();
      }
      ind++;
      continue;
    }
    // take the module with smallest cost, and note its index as indActive_
    int preOn = moduleCost_[ind].whichChild();
    if (children_nodes_[preOn]->status() == NodeStatus::FAILURE) {
      // if a solution is feasible, it should NOT has ES returned FAILURE.
      // This rule-out is just to be safe.
      ind++;
      continue;
    }
    indActive_ = preOn;
    break;
  }
  return NodeStatus::RUNNING;
}

NodeStatus DeciderNode::tick() {
  NodeStatus childStatus;
  if (status() == NodeStatus::IDLE) {
    // no module is running yet. tick estimator of each module by sending a tick signal to each idle
    // module. retry counter resets on every halt.
    nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
    childStatus = turnOnBest();
    // SUCCESS: return SUCCESS since at least one candidate already reported completion;
    // FAILURE: return FAILURE since no candidate fits.
    if (childStatus != NodeStatus::RUNNING) return childStatus;
    setStatus(NodeStatus::RUNNING);
    nRetry_[indActive_] += 1;
  }
  // one module is already running. tick until its status to turn SUCCESS or FAILURE, and check EV
  // status.
  childStatus = children_nodes_[indActive_]->executeTick();
  // update its EV reading after each tick.
  ReFRESH_Module* tryConvert = dynamic_cast<ReFRESH_Module*>(children_nodes_[indActive_]);
  if (tryConvert == nullptr) {
    // not a refresh module. if node status is not-failure then make its cost 1-eps (least
    // considered)
    float pCost_, rCost_;
    if (childStatus == NodeStatus::FAILURE) {
      pCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
      rCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
    } else {
      pCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
      rCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
    }
    activeModuleCost_ = ReFRESH_Cost(indActive_, pCost_, rCost_);
  } else {
    std::tuple<NodeStatus, float, float> evCost = tryConvert->evaluate();
    activeModuleCost_ = ReFRESH_Cost(indActive_, std::get<1>(evCost), std::get<2>(evCost));
  }

  // No need to use the weights since we are not ranking...
  // float pWeight_ = getInput<float>("performance_weight").value();
  // float rWeight_ = getInput<float>("resource_weight").value();
  // activeModuleCost_.setWeights(pWeight_, rWeight_);

  // Upon terminal state, check EV cost reading.
  // If any child module returns SUCCESS, return SUCCESS.
  // SUCCESS and FAILURE are terminal states, therefore no need to halt separately.
  if (childStatus == NodeStatus::SUCCESS) {
    nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
    return NodeStatus::SUCCESS;
  }
  // Something went wrong (causes are handled inside each ReFRESH module), trigger reconfig.
  if (childStatus == NodeStatus::FAILURE) {
    // tick inactive modules once, update and sort moduleCost_
    int lastRun = indActive_;
    NodeStatus bestResultsInCandidates = turnOnBest();
    // indActive_ has been changed, if a viable solution is found. We need to verify it.
    // if status is not FAILURE, the result is valid.
    // get policy when candidate can not provide a satisfactory alternative.
    if (bestResultsInCandidates == NodeStatus::FAILURE) {
      // select between useBestPossibleAlt, and keepCurrentConfig.
      bool keepCurrent = getInput<bool>("fallback_no_reconfig").value();
      // setActive and tick the min-cost and not last-run module
      if (keepCurrent)
        indActive_ = lastRun;
      else
        indActive_ = bestPossible_;
    }

    nRetry_[indActive_] += 1;
    // called after each reconfig
    if (nRetry_[indActive_] > getInput<unsigned int>("retries").value()) {
      nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
      return NodeStatus::FAILURE;
    }
  }
  return NodeStatus::RUNNING;
}

void ReactorNode::halt() {
  haltChild(indActive_);
  // retry counter resets on every halt.
  nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
  indActive_ = -1;
  setStatus(NodeStatus::IDLE);
}

NodeStatus ReactorNode::turnOnBestMitigation() {
  std::vector<ReFRESH_Cost> moduleCost_;
  size_t ind;
  float pWeight_ = getInput<float>("performance_weight").value();
  float rWeight_ = getInput<float>("resource_weight").value();
  unsigned int retries = getInput<unsigned int>("retries").value();
  for (ind = 0; ind < childrenCount() - 1; ind++) {
    // this list is for inactive modules.
    if (ind == indActive_) continue;
    // this module has exceeded max retry limits. Skip it.
    if (nRetry_[ind] > retries) continue;

    ReFRESH_Module* tryConvert = dynamic_cast<ReFRESH_Module*>(children_nodes_[ind]);
    if (tryConvert == nullptr) {
      // not a refresh module. if node status is not-failure then make its cost 1-eps (least
      // considered)
      float pCost_, rCost_;
      NodeStatus childStatus = children_nodes_[ind]->executeTick();

      switch (childStatus) {
        case NodeStatus::SUCCESS:
          // this is a sequence node, anyway.
          break;

        case NodeStatus::FAILURE:
          pCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
          rCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
          moduleCost_.push_back(ReFRESH_Cost(ind, pCost_, rCost_));
          moduleCost_.back().setWeights(pWeight_, rWeight_);
          break;

        case NodeStatus::RUNNING:
          // not a refresh module. if node status is not-failure then make its cost 1-eps (least
          // considered)
          pCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
          rCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
          haltChild(ind);
          moduleCost_.push_back(ReFRESH_Cost(ind, pCost_, rCost_));
          moduleCost_.back().setWeights(pWeight_, rWeight_);
          break;

        default:
          throw BT::LogicError("A child node must never return IDLE");
          break;
      }
      continue;
    }
    std::tuple<NodeStatus, float, float> esCost = tryConvert->estimate();
    // ES returned FAILURE (hard failure), skipping
    if (std::get<0>(esCost) == NodeStatus::FAILURE) continue;
    moduleCost_.push_back(ReFRESH_Cost(ind, std::get<1>(esCost), std::get<2>(esCost)));
    moduleCost_.back().setWeights(pWeight_, rWeight_);
  }
  // sort moduleCost_
  std::stable_sort(moduleCost_.begin(), moduleCost_.end());
  // check feasibility in cost-ascending order
  ind = 0;
  bestPossible_ = childrenCount() - 1;
  bool bestPossibleSet = false;
  while (true) {
    if (ind >= moduleCost_.size()) {
      // no candidate satisfies the requirements
      return NodeStatus::FAILURE;
    }
    if (!moduleCost_[ind].feasible()) {
      if (!bestPossibleSet && moduleCost_[ind].resourceFeasible()) {
        bestPossibleSet = true;
        bestPossible_ = moduleCost_[ind].whichChild();
      }
      ind++;
      continue;
    }
    // take the module with smallest cost, and note its index as indActive_
    int preOn = moduleCost_[ind].whichChild();
    if (children_nodes_[preOn]->status() == NodeStatus::FAILURE) {
      // if a solution is feasible, it should NOT has ES returned FAILURE.
      // This rule-out is just to be safe.
      ind++;
      continue;
    }
    indActive_ = preOn;
    break;
  }
  return NodeStatus::RUNNING;
}

NodeStatus ReactorNode::turnOnNominal() {
  float pWeight_ = getInput<float>("performance_weight").value();
  float rWeight_ = getInput<float>("resource_weight").value();
  unsigned int retries = getInput<unsigned int>("retries").value();

  // this list is for inactive modules.
  if (indActive_ = childrenCount() - 1) return NodeStatus::RUNNING;
  // this module has exceeded max retry limits. Skip it.
  if (nRetry_.back() > retries) return NodeStatus::FAILURE;

  ReFRESH_Module* tryConvert = dynamic_cast<ReFRESH_Module*>(children_nodes_.back());
  if (tryConvert == nullptr) {
    // not a refresh module. if node status is not-failure then make its cost 1-eps (least
    // considered)
    float pCost_, rCost_;
    NodeStatus childStatus = tryConvert->executeTick();
    switch (childStatus) {
      case NodeStatus::SUCCESS:
        return NodeStatus::SUCCESS;
        break;

      case NodeStatus::FAILURE:
        pCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
        rCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
        activeModuleCost_ = ReFRESH_Cost(childrenCount() - 1, pCost_, rCost_);
        return NodeStatus::FAILURE;
        break;

      case NodeStatus::RUNNING:
        // not a refresh module. if node status is not-failure then make its cost 1-eps (least
        // considered)
        pCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
        rCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
        activeModuleCost_ = ReFRESH_Cost(childrenCount() - 1, pCost_, rCost_);
        return NodeStatus::RUNNING;
        break;

      default:
        throw BT::LogicError("A child node must never return IDLE");
        break;
    }
  }
  std::tuple<NodeStatus, float, float> esCost = tryConvert->estimate();
  // ES returned FAILURE (hard failure), skipping
  if (std::get<0>(esCost) == NodeStatus::FAILURE) return NodeStatus::FAILURE;
  if (std::get<1>(esCost) >= 1.0 || std::get<2>(esCost) >= 1.0) return NodeStatus::FAILURE;
  return NodeStatus::RUNNING;
}

NodeStatus ReactorNode::tick() {
  NodeStatus childStatus;
  if (status() == NodeStatus::IDLE) {
    // retry counter resets on every halt.
    nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
    // no module is running yet. First check if the entry condition of the last module is satisfied.
    setStatus(NodeStatus::RUNNING);
    // check entry condition of last module
    childStatus = turnOnNominal();
    // check mitigation solution if entry is not satisfied
    if (childStatus == NodeStatus::FAILURE) {
      childStatus = turnOnBestMitigation();
      // no mitigation is naturally found, try fallback
      if (childStatus == NodeStatus::FAILURE) {
        // select between useBestPossibleAlt, and keepCurrentConfig.
        bool keepCurrent = getInput<bool>("fallback_no_reconfig").value();
        // setActive and tick the min-cost and not last-run module
        if (keepCurrent)
          return NodeStatus::FAILURE;
        else
          indActive_ = bestPossible_;
      }
    }
    // TODO: if turnOnNominal returned SUCCESS?
  }

  // one module is already running. tick until its status to turn SUCCESS or FAILURE, and check EV
  // status.
  childStatus = children_nodes_[indActive_]->executeTick();

  // update its EV reading after each tick.
  ReFRESH_Module* tryConvert = dynamic_cast<ReFRESH_Module*>(children_nodes_[indActive_]);
  if (tryConvert == nullptr) {
    // not a refresh module. if node status is not-failure then make its cost 1-eps (least
    // considered)
    float pCost_, rCost_;
    if (childStatus == NodeStatus::FAILURE) {
      pCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
      rCost_ = ReFRESH_Cost::BOUNDARY_REJECT;
    } else {
      pCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
      rCost_ = ReFRESH_Cost::BOUNDARY_ACCEPT;
    }
    activeModuleCost_ = ReFRESH_Cost(indActive_, pCost_, rCost_);
  } else {
    std::tuple<NodeStatus, float, float> evCost = tryConvert->evaluate();
    activeModuleCost_ = ReFRESH_Cost(indActive_, std::get<1>(evCost), std::get<2>(evCost));
  }

  // check mitigation solution if entry is not satisfied
  if (childStatus == NodeStatus::FAILURE) {
    int lastRun = indActive_;
    childStatus = turnOnBestMitigation();
    // no mitigation is naturally found, try fallback
    if (childStatus == NodeStatus::FAILURE) {
      // select between useBestPossibleAlt, and keepCurrentConfig.
      bool keepCurrent = getInput<bool>("fallback_no_reconfig").value();
      // setActive and tick the min-cost and not last-run module
      if (keepCurrent)
        indActive_ = lastRun;
      else
        indActive_ = bestPossible_;
    }
    // Mitigation is RUNNING
    nRetry_[indActive_] += 1;
    // called after each reconfig
    if (nRetry_[indActive_] > getInput<unsigned int>("retries").value()) {
      nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
      return NodeStatus::FAILURE;
    }
  }
  // current module returned SUCCESS
  else if (childStatus == NodeStatus::SUCCESS) {
    // this is the last module
    if (indActive_ == childrenCount() - 1) return NodeStatus::SUCCESS;
    // else, check if last module entrance condition is met
    childStatus = turnOnNominal();
    // check mitigation solution if entry is not satisfied
    if (childStatus == NodeStatus::FAILURE) {
      childStatus = turnOnBestMitigation();
      // no mitigation is naturally found, try fallback
      if (childStatus == NodeStatus::FAILURE) {
        // select between useBestPossibleAlt, and keepCurrentConfig.
        bool keepCurrent = getInput<bool>("fallback_no_reconfig").value();
        // setActive and tick the min-cost and not last-run module
        if (keepCurrent)
          indActive_ = childrenCount() - 1;
        else
          indActive_ = bestPossible_;
      }
      // Mitigation is RUNNING
      nRetry_[indActive_] += 1;
      // called after each reconfig
      if (nRetry_[indActive_] > getInput<unsigned int>("retries").value()) {
        nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
        return NodeStatus::FAILURE;
      }
    }
    // TODO: if turnOnNominal returned SUCCESS?
  }

  // current module is RUNNING
  return NodeStatus::RUNNING;
}
}  // namespace ReFRESH_BT
